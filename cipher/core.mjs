import { GifReader, GifWriter } from "./vendor/omggif.mjs";

const textEncoder = new TextEncoder();
const textDecoder = new TextDecoder("utf-8", { fatal: true });

const CONTENT_MAGIC = textEncoder.encode("TCF1");
const ENVELOPE_MAGIC = textEncoder.encode("TCG1");
const FRAME_MAGIC = textEncoder.encode("CGF2");
const PROTOCOL_VERSION = 2;
const KDF_PBKDF2_SHA256 = 1;
const FLAG_GZIP = 1;
const CONTENT_HEADER_LENGTH = 13;
const ENVELOPE_HEADER_LENGTH = 44;
const FRAME_HEADER_LENGTH = 40;
const FRAME_WIDTH = 320;
const FRAME_HEIGHT = 320;
const FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT;
const DATA_TOP = 32;
const CELL_SIZE = 2;
const DATA_COLUMNS = FRAME_WIDTH / CELL_SIZE;
const DATA_ROWS = (FRAME_HEIGHT - DATA_TOP) / CELL_SIZE;
const DATA_BYTES = Math.floor((DATA_COLUMNS * DATA_ROWS * 2) / 8);
const CHUNK_BYTES = DATA_BYTES - FRAME_HEADER_LENGTH;
const MIN_ANIMATION_FRAMES = 3;
const FRAME_REPEAT = 2;
const MAX_CHUNKS = 4096;
const PBKDF2_ITERATIONS = 600_000;
const MAX_INPUT_BYTES = 1 * 1024 * 1024;

const COLOR_LEVELS = [32, 96, 160, 224];
const VARIANT_DELTAS = [
  [0, 0, 0],
  [18, 0, 0],
  [0, 18, 0],
  [0, 0, 18],
];

const colorPalette = [];
for (let base = 0; base < 64; base++) {
  const red = COLOR_LEVELS[(base >> 4) & 3];
  const green = COLOR_LEVELS[(base >> 2) & 3];
  const blue = COLOR_LEVELS[base & 3];
  for (const [dr, dg, db] of VARIANT_DELTAS) {
    colorPalette.push(
      ((Math.min(255, red + dr) << 16) |
        (Math.min(255, green + dg) << 8) |
        Math.min(255, blue + db)) >>> 0,
    );
  }
}

const crcTable = new Uint32Array(256);
for (let n = 0; n < 256; n++) {
  let value = n;
  for (let bit = 0; bit < 8; bit++) {
    value = value & 1 ? 0xedb88320 ^ (value >>> 1) : value >>> 1;
  }
  crcTable[n] = value >>> 0;
}

export const limits = Object.freeze({
  maxInputBytes: MAX_INPUT_BYTES,
  frameWidth: FRAME_WIDTH,
  frameHeight: FRAME_HEIGHT,
  chunkBytes: CHUNK_BYTES,
  pbkdf2Iterations: PBKDF2_ITERATIONS,
});

function assertCrypto() {
  if (!globalThis.crypto?.subtle || !globalThis.crypto?.getRandomValues) {
    throw new Error("当前浏览器不支持安全加密接口。");
  }
  return globalThis.crypto;
}

function bytesEqual(left, right) {
  if (left.length !== right.length) return false;
  let difference = 0;
  for (let index = 0; index < left.length; index++) {
    difference |= left[index] ^ right[index];
  }
  return difference === 0;
}

function hasMagic(bytes, magic, offset = 0) {
  if (bytes.length < offset + magic.length) return false;
  for (let index = 0; index < magic.length; index++) {
    if (bytes[offset + index] !== magic[index]) return false;
  }
  return true;
}

function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) crc = crcTable[(crc ^ byte) & 0xff] ^ (crc >>> 8);
  return (crc ^ 0xffffffff) >>> 0;
}

function safeFileName(name) {
  const baseName = String(name || "").split(/[\\/]/).pop() || "";
  const cleaned = baseName.replace(/[\u0000-\u001f\u007f]/g, "").trim();
  return cleaned && cleaned !== "." && cleaned !== ".." ? cleaned : "message.bin";
}

function normalizeContent(content) {
  const kind = content?.kind === "text" ? "text" : "file";
  const data = content?.data instanceof Uint8Array
    ? content.data
    : new Uint8Array(content?.data || 0);
  if (data.length === 0) throw new Error("内容不能为空。");
  if (data.length > MAX_INPUT_BYTES) throw new Error("第一版最多加密 1 MB 内容。");

  if (kind === "text") {
    return {
      kind,
      name: "",
      mime: "text/plain;charset=utf-8",
      data,
    };
  }

  return {
    kind,
    name: safeFileName(content?.name),
    mime: String(content?.mime || "application/octet-stream"),
    data,
  };
}

function packContent(content) {
  const normalized = normalizeContent(content);
  const nameBytes = textEncoder.encode(normalized.name);
  const mimeBytes = textEncoder.encode(normalized.mime);
  if (nameBytes.length > 0xffff || mimeBytes.length > 0xffff) {
    throw new Error("文件名或文件类型过长。");
  }

  const packed = new Uint8Array(
    CONTENT_HEADER_LENGTH + nameBytes.length + mimeBytes.length + normalized.data.length,
  );
  const view = new DataView(packed.buffer);
  packed.set(CONTENT_MAGIC, 0);
  view.setUint8(4, normalized.kind === "text" ? 1 : 2);
  view.setUint16(5, nameBytes.length, true);
  view.setUint16(7, mimeBytes.length, true);
  view.setUint32(9, normalized.data.length, true);
  packed.set(nameBytes, CONTENT_HEADER_LENGTH);
  packed.set(mimeBytes, CONTENT_HEADER_LENGTH + nameBytes.length);
  packed.set(normalized.data, CONTENT_HEADER_LENGTH + nameBytes.length + mimeBytes.length);
  return packed;
}

function unpackContent(packed) {
  if (packed.length < CONTENT_HEADER_LENGTH || !hasMagic(packed, CONTENT_MAGIC)) {
    throw new Error("解密内容的格式无效。");
  }
  const view = new DataView(packed.buffer, packed.byteOffset, packed.byteLength);
  const kindValue = view.getUint8(4);
  const nameLength = view.getUint16(5, true);
  const mimeLength = view.getUint16(7, true);
  const dataLength = view.getUint32(9, true);
  const dataOffset = CONTENT_HEADER_LENGTH + nameLength + mimeLength;
  if (
    (kindValue !== 1 && kindValue !== 2) ||
    dataLength === 0 ||
    dataLength > MAX_INPUT_BYTES ||
    dataOffset + dataLength !== packed.length
  ) {
    throw new Error("解密内容的长度无效。");
  }

  const name = textDecoder.decode(
    packed.subarray(CONTENT_HEADER_LENGTH, CONTENT_HEADER_LENGTH + nameLength),
  );
  const mime = textDecoder.decode(
    packed.subarray(CONTENT_HEADER_LENGTH + nameLength, dataOffset),
  );
  return {
    kind: kindValue === 1 ? "text" : "file",
    name: kindValue === 1 ? "" : safeFileName(name),
    mime: mime || "application/octet-stream",
    data: packed.slice(dataOffset),
  };
}

async function transformBytes(bytes, transform) {
  const stream = new Blob([bytes]).stream().pipeThrough(transform);
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

async function compressContent(bytes) {
  if (typeof CompressionStream === "undefined" || bytes.length < 512) {
    return { bytes, compressed: false };
  }
  const compressed = await transformBytes(bytes, new CompressionStream("gzip"));
  return compressed.length + 64 < bytes.length
    ? { bytes: compressed, compressed: true }
    : { bytes, compressed: false };
}

async function decompressContent(bytes) {
  if (typeof DecompressionStream === "undefined") {
    throw new Error("当前浏览器无法解压这条消息。");
  }
  const reader = new Blob([bytes])
    .stream()
    .pipeThrough(new DecompressionStream("gzip"))
    .getReader();
  const chunks = [];
  const limit = MAX_INPUT_BYTES + 131072;
  let total = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    total += value.length;
    if (total > limit) {
      await reader.cancel();
      throw new Error("解压内容超过安全限制。");
    }
    chunks.push(value);
  }
  const decompressed = new Uint8Array(total);
  let offset = 0;
  for (const chunk of chunks) {
    decompressed.set(chunk, offset);
    offset += chunk.length;
  }
  return decompressed;
}

async function deriveKey(password, salt, iterations, usage) {
  const cryptoApi = assertCrypto();
  const passwordBytes = textEncoder.encode(String(password).normalize("NFKC"));
  const baseKey = await cryptoApi.subtle.importKey(
    "raw",
    passwordBytes,
    "PBKDF2",
    false,
    ["deriveKey"],
  );
  return cryptoApi.subtle.deriveKey(
    { name: "PBKDF2", hash: "SHA-256", salt, iterations },
    baseKey,
    { name: "AES-GCM", length: 256 },
    false,
    [usage],
  );
}

async function encryptContent(content, password, onProgress) {
  if (String(password).length < 8) throw new Error("密码至少需要 8 个字符。");
  onProgress?.("正在封装内容…", 0.08);
  const packed = packContent(content);
  const compressed = await compressContent(packed);
  const cryptoApi = assertCrypto();
  const salt = cryptoApi.getRandomValues(new Uint8Array(16));
  const iv = cryptoApi.getRandomValues(new Uint8Array(12));
  const expectedCipherLength = compressed.bytes.length + 16;
  const header = new Uint8Array(ENVELOPE_HEADER_LENGTH);
  const view = new DataView(header.buffer);
  header.set(ENVELOPE_MAGIC, 0);
  view.setUint8(4, PROTOCOL_VERSION);
  view.setUint8(5, compressed.compressed ? FLAG_GZIP : 0);
  view.setUint8(6, KDF_PBKDF2_SHA256);
  view.setUint8(7, 0);
  view.setUint32(8, PBKDF2_ITERATIONS, true);
  header.set(salt, 12);
  header.set(iv, 28);
  view.setUint32(40, expectedCipherLength, true);

  onProgress?.("正在派生加密密钥…", 0.2);
  const key = await deriveKey(password, salt, PBKDF2_ITERATIONS, "encrypt");
  onProgress?.("正在加密…", 0.42);
  const cipherBuffer = await cryptoApi.subtle.encrypt(
    { name: "AES-GCM", iv, additionalData: header, tagLength: 128 },
    key,
    compressed.bytes,
  );
  const cipher = new Uint8Array(cipherBuffer);
  if (cipher.length !== expectedCipherLength) throw new Error("加密结果长度异常。");
  const envelope = new Uint8Array(header.length + cipher.length);
  envelope.set(header, 0);
  envelope.set(cipher, header.length);
  return { envelope, compressed: compressed.compressed };
}

async function decryptContent(envelope, password, onProgress) {
  if (envelope.length < ENVELOPE_HEADER_LENGTH + 16 || !hasMagic(envelope, ENVELOPE_MAGIC)) {
    throw new Error("GIF 中的加密信封无效。");
  }
  const header = envelope.slice(0, ENVELOPE_HEADER_LENGTH);
  const view = new DataView(header.buffer);
  const version = view.getUint8(4);
  const flags = view.getUint8(5);
  const kdf = view.getUint8(6);
  const iterations = view.getUint32(8, true);
  const cipherLength = view.getUint32(40, true);
  if (
    version !== PROTOCOL_VERSION ||
    flags & ~FLAG_GZIP ||
    kdf !== KDF_PBKDF2_SHA256 ||
    iterations < 100_000 ||
    iterations > 2_000_000 ||
    cipherLength !== envelope.length - ENVELOPE_HEADER_LENGTH
  ) {
    throw new Error("GIF 使用了不支持的加密协议。");
  }

  const salt = header.slice(12, 28);
  const iv = header.slice(28, 40);
  onProgress?.("正在派生解密密钥…", 0.62);
  const key = await deriveKey(password, salt, iterations, "decrypt");
  let plain;
  try {
    plain = new Uint8Array(
      await assertCrypto().subtle.decrypt(
        { name: "AES-GCM", iv, additionalData: header, tagLength: 128 },
        key,
        envelope.subarray(ENVELOPE_HEADER_LENGTH),
      ),
    );
  } catch {
    throw new Error("密码错误，或 GIF 数据已经损坏。");
  }
  onProgress?.("正在验证内容…", 0.86);
  if (flags & FLAG_GZIP) plain = await decompressContent(plain);
  return unpackContent(plain);
}

function writeFrameHeader(target, messageId, chunkIndex, chunkCount, chunkLength, totalLength, crc) {
  const view = new DataView(target.buffer, target.byteOffset, FRAME_HEADER_LENGTH);
  target.set(FRAME_MAGIC, 0);
  view.setUint8(4, PROTOCOL_VERSION);
  view.setUint8(5, 0);
  view.setUint16(6, FRAME_HEADER_LENGTH, true);
  target.set(messageId, 8);
  view.setUint32(16, chunkIndex, true);
  view.setUint32(20, chunkCount, true);
  view.setUint32(24, chunkLength, true);
  view.setUint32(28, totalLength, true);
  view.setUint32(32, crc, true);
  view.setUint32(36, crc32(target.subarray(0, 36)), true);
}

function baseColorIndex(red, green, blue) {
  const nearest = (value) => {
    let best = 0;
    let distance = Infinity;
    for (let index = 0; index < COLOR_LEVELS.length; index++) {
      const current = Math.abs(value - COLOR_LEVELS[index]);
      if (current < distance) {
        distance = current;
        best = index;
      }
    }
    return best;
  };
  return (nearest(red) << 4) | (nearest(green) << 2) | nearest(blue);
}

function paletteColor(base, variant) {
  const red = COLOR_LEVELS[(base >> 4) & 3];
  const green = COLOR_LEVELS[(base >> 2) & 3];
  const blue = COLOR_LEVELS[base & 3];
  const [dr, dg, db] = VARIANT_DELTAS[variant];
  return [Math.min(255, red + dr), Math.min(255, green + dg), Math.min(255, blue + db)];
}

function nearestVariant(red, green, blue) {
  const base = baseColorIndex(red, green, blue);
  let best = 0;
  let distance = Infinity;
  for (let variant = 0; variant < 4; variant++) {
    const [r, g, b] = paletteColor(base, variant);
    const current = (red - r) ** 2 + (green - g) ** 2 + (blue - b) ** 2;
    if (current < distance) {
      distance = current;
      best = variant;
    }
  }
  return best;
}

function defaultCoverPixels() {
  const pixels = new Uint8ClampedArray(FRAME_BYTES * 4);
  for (let y = 0; y < FRAME_HEIGHT; y++) {
    for (let x = 0; x < FRAME_WIDTH; x++) {
      const offset = (y * FRAME_WIDTH + x) * 4;
      const tone = ((Math.floor(x / 16) + Math.floor(y / 16)) % 2) * 18;
      pixels[offset] = 28 + tone;
      pixels[offset + 1] = 38 + tone;
      pixels[offset + 2] = 45 + tone;
      pixels[offset + 3] = 255;
    }
  }
  return pixels;
}

function glyphRows(character) {
  const glyphs = {
    C: ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
    E: ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    F: ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    G: ["01110", "10001", "10000", "10111", "10001", "10001", "01110"],
    H: ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    I: ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    P: ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    R: ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
  };
  return glyphs[character] || glyphs[" "];
}

function drawInscription(indexed, text, x, y, scale, colorIndex) {
  for (const character of text) {
    const rows = glyphRows(character);
    for (let row = 0; row < rows.length; row++) {
      for (let column = 0; column < rows[row].length; column++) {
        if (rows[row][column] !== "1") continue;
        for (let dy = 0; dy < scale; dy++) {
          for (let dx = 0; dx < scale; dx++) {
            const px = x + column * scale + dx;
            const py = y + row * scale + dy;
            if (px >= 0 && px < FRAME_WIDTH && py >= 0 && py < DATA_TOP) {
              indexed[py * FRAME_WIDTH + px] = colorIndex;
            }
          }
        }
      }
    }
    x += 6 * scale;
  }
}

function buildFramePixels(coverPixels, frameBytes, frameIndex, animationFrames) {
  const indexed = new Uint8Array(FRAME_BYTES);
  const cover = coverPixels?.length === FRAME_BYTES * 4 ? coverPixels : defaultCoverPixels();
  for (let y = 0; y < FRAME_HEIGHT; y++) {
    for (let x = 0; x < FRAME_WIDTH; x++) {
      const source = (y * FRAME_WIDTH + x) * 4;
      const red = cover[source];
      const green = cover[source + 1];
      const blue = cover[source + 2];
      indexed[y * FRAME_WIDTH + x] = baseColorIndex(red, green, blue) * 4;
    }
  }

  const dark = 0;
  const light = (3 << 4 | 3 << 2 | 3) * 4;
  for (let y = 0; y < DATA_TOP; y++) {
    indexed.fill(dark, y * FRAME_WIDTH, (y + 1) * FRAME_WIDTH);
  }
  drawInscription(indexed, "CIPHER GIF", 14, 9, 2, light);
  const progressWidth = Math.max(1, Math.round((FRAME_WIDTH - 28) * ((frameIndex + 1) / animationFrames)));
  indexed.fill(light, (DATA_TOP - 5) * FRAME_WIDTH + 14, (DATA_TOP - 5) * FRAME_WIDTH + 14 + progressWidth);

  const byteCount = frameBytes.length;
  const symbolCount = DATA_COLUMNS * DATA_ROWS;
  for (let cell = 0; cell < symbolCount; cell++) {
    const byteIndex = Math.floor(cell / 4);
    const symbolOffset = cell % 4;
    const symbol = byteIndex < byteCount
      ? (frameBytes[byteIndex] >> (6 - symbolOffset * 2)) & 3
      : 0;
    const base = baseColorIndex(
      cover[((DATA_TOP + Math.floor(cell / DATA_COLUMNS) * CELL_SIZE) * FRAME_WIDTH +
        (cell % DATA_COLUMNS) * CELL_SIZE) * 4],
      cover[((DATA_TOP + Math.floor(cell / DATA_COLUMNS) * CELL_SIZE) * FRAME_WIDTH +
        (cell % DATA_COLUMNS) * CELL_SIZE) * 4 + 1],
      cover[((DATA_TOP + Math.floor(cell / DATA_COLUMNS) * CELL_SIZE) * FRAME_WIDTH +
        (cell % DATA_COLUMNS) * CELL_SIZE) * 4 + 2],
    );
    // Each logical cell repeats one symbol across 2×2 pixels for rescaling tolerance.
    const colorIndex = base * 4 + symbol;
    const cellX = (cell % DATA_COLUMNS) * CELL_SIZE;
    const cellY = DATA_TOP + Math.floor(cell / DATA_COLUMNS) * CELL_SIZE;
    for (let dy = 0; dy < CELL_SIZE; dy++) {
      indexed.fill(colorIndex, (cellY + dy) * FRAME_WIDTH + cellX, (cellY + dy) * FRAME_WIDTH + cellX + CELL_SIZE);
    }
  }
  return indexed;
}

function parseFrameBytes(frameBytes) {
  if (frameBytes.length < FRAME_HEADER_LENGTH || !hasMagic(frameBytes, FRAME_MAGIC)) return null;
  const view = new DataView(frameBytes.buffer, frameBytes.byteOffset, frameBytes.byteLength);
  if (
    view.getUint8(4) !== PROTOCOL_VERSION ||
    view.getUint16(6, true) !== FRAME_HEADER_LENGTH ||
    view.getUint32(36, true) !== crc32(frameBytes.subarray(0, 36))
  ) return null;
  const chunkIndex = view.getUint32(16, true);
  const chunkCount = view.getUint32(20, true);
  const chunkLength = view.getUint32(24, true);
  const totalLength = view.getUint32(28, true);
  if (
    chunkCount === 0 ||
    chunkCount > MAX_CHUNKS ||
    chunkIndex >= chunkCount ||
    chunkLength === 0 ||
    chunkLength > CHUNK_BYTES ||
    totalLength === 0 ||
    totalLength > MAX_INPUT_BYTES + 262144
  ) return null;
  const chunk = frameBytes.slice(FRAME_HEADER_LENGTH, FRAME_HEADER_LENGTH + chunkLength);
  if (view.getUint32(32, true) !== crc32(chunk)) return null;
  return { messageId: frameBytes.slice(8, 16), chunkIndex, chunkCount, totalLength, chunk };
}

function encodeEnvelopeAsGif(envelope, options = {}) {
  const onProgress = typeof options === "function" ? options : options.onProgress;
  const coverPixels = typeof options === "function" ? null : options.coverPixels;
  const chunkCount = Math.ceil(envelope.length / CHUNK_BYTES);
  if (chunkCount > MAX_CHUNKS) throw new Error("消息需要的 GIF 帧数过多。");
  const animationFrames = Math.max(MIN_ANIMATION_FRAMES, chunkCount * FRAME_REPEAT);
  const estimatedBytes = 8192 + animationFrames * (FRAME_BYTES * 2 + 8192);
  const output = new Uint8Array(estimatedBytes);
  const writer = new GifWriter(output, FRAME_WIDTH, FRAME_HEIGHT, { loop: 0, palette: colorPalette });
  const messageId = assertCrypto().getRandomValues(new Uint8Array(8));
  for (let frameIndex = 0; frameIndex < animationFrames; frameIndex++) {
    const chunkIndex = Math.floor(frameIndex / FRAME_REPEAT) % chunkCount;
    const start = chunkIndex * CHUNK_BYTES;
    const chunk = envelope.subarray(start, Math.min(start + CHUNK_BYTES, envelope.length));
    const frameBytes = new Uint8Array(DATA_BYTES);
    writeFrameHeader(frameBytes, messageId, chunkIndex, chunkCount, chunk.length, envelope.length, crc32(chunk));
    frameBytes.set(chunk, FRAME_HEADER_LENGTH);
    const pixels = buildFramePixels(coverPixels, frameBytes, frameIndex, animationFrames);
    writer.addFrame(0, 0, FRAME_WIDTH, FRAME_HEIGHT, pixels, { delay: 12, disposal: 1 });
    onProgress?.("正在铭刻图片帧…", 0.48 + 0.47 * ((frameIndex + 1) / animationFrames));
  }
  const length = writer.end();
  if (length > output.length) throw new Error("GIF 输出缓冲区不足。");
  return {
    blob: new Blob([output.slice(0, length)], { type: "image/gif" }),
    frameCount: animationFrames,
    dataFrameCount: chunkCount,
    messageId: [...messageId].map((byte) => byte.toString(16).padStart(2, "0")).join(""),
  };
}

export function makeDefaultCover() {
  return defaultCoverPixels();
}

export async function createCipherGif(content, password, options) {
  const settings = typeof options === "function" ? { onProgress: options } : (options || {});
  const encrypted = await encryptContent(content, password, settings.onProgress);
  const gif = encodeEnvelopeAsGif(encrypted.envelope, settings);
  settings.onProgress?.("加密铭文 GIF 已生成。", 1);
  return { ...gif, compressed: encrypted.compressed };
}

function decodeFrameBytes(rgba, width, height) {
  if (width < 80 || height < 80) return null;
  const dataTop = height * (DATA_TOP / FRAME_HEIGHT);
  const cellWidth = width / DATA_COLUMNS;
  const cellHeight = (height - dataTop) / DATA_ROWS;
  const frameBytes = new Uint8Array(DATA_BYTES);
  const symbols = new Uint8Array(DATA_COLUMNS * DATA_ROWS);
  for (let cell = 0; cell < symbols.length; cell++) {
    const cellX = cell % DATA_COLUMNS;
    const cellY = Math.floor(cell / DATA_COLUMNS);
    const startX = Math.floor((cellX + 0.25) * cellWidth);
    const endX = Math.max(startX + 1, Math.ceil((cellX + 0.75) * cellWidth));
    const startY = Math.floor(dataTop + (cellY + 0.25) * cellHeight);
    const endY = Math.max(startY + 1, Math.ceil(dataTop + (cellY + 0.75) * cellHeight));
    let red = 0;
    let green = 0;
    let blue = 0;
    let count = 0;
    for (let y = startY; y < Math.min(height, endY); y++) {
      for (let x = startX; x < Math.min(width, endX); x++) {
        const offset = (y * width + x) * 4;
        red += rgba[offset];
        green += rgba[offset + 1];
        blue += rgba[offset + 2];
        count++;
      }
    }
    symbols[cell] = nearestVariant(red / count, green / count, blue / count);
  }
  for (let cell = 0; cell < symbols.length; cell++) {
    const byteIndex = Math.floor(cell / 4);
    frameBytes[byteIndex] |= symbols[cell] << (6 - (cell % 4) * 2);
  }
  return frameBytes;
}

export function extractEnvelopeFromGif(gifBytes, onProgress) {
  const bytes = gifBytes instanceof Uint8Array ? gifBytes : new Uint8Array(gifBytes);
  let reader;
  try {
    reader = new GifReader(bytes);
  } catch {
    throw new Error("无法读取这个 GIF 文件。");
  }
  const frameTotal = reader.numFrames();
  if (frameTotal === 0) throw new Error("GIF 中没有动画帧。");
  const rgba = new Uint8Array(reader.width * reader.height * 4);
  let messageId = null;
  let chunkCount = 0;
  let totalLength = 0;
  const chunks = new Map();
  let validFrames = 0;
  for (let frameIndex = 0; frameIndex < frameTotal; frameIndex++) {
    reader.decodeAndBlitFrameRGBA(frameIndex, rgba);
    const decoded = decodeFrameBytes(rgba, reader.width, reader.height);
    const parsed = decoded && parseFrameBytes(decoded);
    if (!parsed) continue;
    if (!messageId) {
      messageId = parsed.messageId;
      chunkCount = parsed.chunkCount;
      totalLength = parsed.totalLength;
    } else if (!bytesEqual(messageId, parsed.messageId) || chunkCount !== parsed.chunkCount || totalLength !== parsed.totalLength) {
      throw new Error("GIF 中混入了不同消息的帧。");
    }
    const previous = chunks.get(parsed.chunkIndex);
    if (previous && !bytesEqual(previous, parsed.chunk)) throw new Error("GIF 中存在冲突的数据帧。");
    chunks.set(parsed.chunkIndex, parsed.chunk);
    validFrames++;
    onProgress?.(`正在读取 GIF 帧 ${frameIndex + 1}/${frameTotal}…`, 0.45 * ((frameIndex + 1) / frameTotal));
  }
  if (!messageId || validFrames === 0) throw new Error("GIF 中没有找到 Cipher GIF 铭文。");
  if (chunks.size !== chunkCount) throw new Error(`GIF 数据不完整：取得 ${chunks.size}/${chunkCount} 个分片。`);
  const envelope = new Uint8Array(totalLength);
  let offset = 0;
  for (let chunkIndex = 0; chunkIndex < chunkCount; chunkIndex++) {
    const chunk = chunks.get(chunkIndex);
    if (!chunk) throw new Error(`缺少第 ${chunkIndex + 1} 个数据分片。`);
    if (offset + chunk.length > envelope.length) throw new Error("GIF 分片长度超过消息长度。");
    envelope.set(chunk, offset);
    offset += chunk.length;
  }
  if (offset !== envelope.length) throw new Error("GIF 分片总长度与消息不一致。");
  return { envelope, frameTotal, dataFrameCount: chunkCount, validFrames };
}

export async function decodeCipherGif(gifBytes, password, onProgress) {
  const extracted = extractEnvelopeFromGif(gifBytes, onProgress);
  const content = await decryptContent(extracted.envelope, password, onProgress);
  onProgress?.("解密和完整性验证通过。", 1);
  return { ...content, ...extracted };
}

export function createRandomPassword() {
  const bytes = assertCrypto().getRandomValues(new Uint8Array(18));
  let binary = "";
  for (const byte of bytes) binary += String.fromCharCode(byte);
  return btoa(binary).replaceAll("+", "-").replaceAll("/", "_").replaceAll("=", "");
}
