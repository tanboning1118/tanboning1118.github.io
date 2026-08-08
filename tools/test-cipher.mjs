import assert from "node:assert/strict";
import test from "node:test";
import {
  createCipherGif,
  createRandomPassword,
  decodeCipherGif,
  extractEnvelopeFromGif,
  makeDefaultCover,
} from "../cipher/core.mjs";

const encoder = new TextEncoder();
const decoder = new TextDecoder();

test("text encrypts to an animated GIF and decrypts exactly", async () => {
  const password = "correct horse battery staple";
  const source = "这是一条 Cipher GIF 测试消息。\nThe bytes must round-trip exactly.";
  const encoded = await createCipherGif(
    { kind: "text", data: encoder.encode(source) },
    password,
    { coverPixels: makeDefaultCover() },
  );
  const gifBytes = new Uint8Array(await encoded.blob.arrayBuffer());
  const recovered = await decodeCipherGif(gifBytes, password);

  assert.equal(encoded.blob.type, "image/gif");
  assert.ok(encoded.frameCount >= 3);
  assert.equal(recovered.kind, "text");
  assert.equal(decoder.decode(recovered.data), source);
});

test("a multi-frame binary file round-trips exactly", async () => {
  const bytes = new Uint8Array(150_000);
  for (let offset = 0; offset < bytes.length; offset += 65536) {
    crypto.getRandomValues(bytes.subarray(offset, Math.min(offset + 65536, bytes.length)));
  }
  const encoded = await createCipherGif(
    { kind: "file", name: "sample.bin", mime: "application/octet-stream", data: bytes },
    "long-enough-test-password",
  );
  const gifBytes = new Uint8Array(await encoded.blob.arrayBuffer());
  const extracted = extractEnvelopeFromGif(gifBytes);
  const recovered = await decodeCipherGif(gifBytes, "long-enough-test-password");

  assert.ok(extracted.dataFrameCount >= 3);
  assert.equal(recovered.kind, "file");
  assert.equal(recovered.name, "sample.bin");
  assert.equal(recovered.mime, "application/octet-stream");
  assert.deepEqual(recovered.data, bytes);
});

test("the wrong password fails authenticated decryption", async () => {
  const encoded = await createCipherGif(
    { kind: "text", data: encoder.encode("secret") },
    "the-right-password",
  );
  const gifBytes = new Uint8Array(await encoded.blob.arrayBuffer());
  await assert.rejects(
    decodeCipherGif(gifBytes, "the-wrong-password"),
    /密码错误|已经损坏/,
  );
});

test("generated passwords have enough entropy and URL-safe characters", () => {
  const first = createRandomPassword();
  const second = createRandomPassword();
  assert.match(first, /^[A-Za-z0-9_-]{24}$/);
  assert.notEqual(first, second);
});
