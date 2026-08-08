import {
  createCipherGif,
  createRandomPassword,
  decodeCipherGif,
  limits,
} from "./core.mjs";

const encoder = new TextEncoder();
const decoder = new TextDecoder();
const $ = (selector) => document.querySelector(selector);

const sendView = $("#send-view");
const receiveView = $("#receive-view");
const modeTabs = [...document.querySelectorAll(".mode-tab")];
const sendForm = $("#send-form");
const receiveForm = $("#receive-form");
const sendButton = $("#encrypt-button");
const decryptButton = $("#decrypt-button");
const secretText = $("#secret-text");
const secretFile = $("#secret-file");
const carrierFile = $("#carrier-file");
const sendPassword = $("#send-password");
const receivePassword = $("#receive-password");
const cipherFile = $("#cipher-file");

let generatedGif = null;
let generatedGifUrl = "";
let receiveSource = null;
let receiveSourceUrl = "";
let decodedFileUrl = "";

function formatBytes(value) {
  if (value < 1024) return `${value} B`;
  if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KB`;
  return `${(value / 1024 / 1024).toFixed(2)} MB`;
}

function switchMode(mode) {
  const sending = mode === "send";
  sendView.hidden = !sending;
  receiveView.hidden = sending;
  for (const tab of modeTabs) {
    const active = tab.dataset.mode === mode;
    tab.classList.toggle("is-active", active);
    tab.setAttribute("aria-selected", String(active));
  }
  window.scrollTo({ top: 0, behavior: "smooth" });
}

function updateStatus(scope, message, fraction = 0, error = false) {
  const root = $(`#${scope}-status`);
  const text = $(`#${scope}-status-text`);
  const progress = $(`#${scope}-progress`);
  root.hidden = false;
  root.classList.toggle("is-error", error);
  text.textContent = message;
  progress.style.width = `${Math.max(0, Math.min(1, fraction)) * 100}%`;
}

function setBusy(button, busy, busyLabel, idleLabel) {
  button.disabled = busy;
  button.textContent = busy ? busyLabel : idleLabel;
}

function updateContentKind() {
  const kind = document.querySelector('input[name="content-kind"]:checked').value;
  $("#text-field").hidden = kind !== "text";
  $("#file-field").hidden = kind !== "file";
  secretText.required = kind === "text";
  secretFile.required = kind === "file";
}

function updateSecretFile(file) {
  if (!file) {
    $("#secret-file-name").textContent = "选择不超过 1 MB 的文件";
    $("#secret-file-size").textContent = "尚未选择";
    return;
  }
  $("#secret-file-name").textContent = file.name;
  $("#secret-file-size").textContent = `${formatBytes(file.size)} · ${file.type || "未知类型"}`;
}

function updateCarrierFile(file) {
  if (!file) {
    $("#carrier-file-name").textContent = "选择一张载体图片";
    $("#carrier-file-size").textContent = "图片会保留在 GIF 每个动画帧中";
    return;
  }
  $("#carrier-file-name").textContent = file.name;
  $("#carrier-file-size").textContent = `${formatBytes(file.size)} · ${file.type || "图片"}`;
}

async function readCarrierPixels(file) {
  if (!file) throw new Error("请先选择一张载体图片。");
  const url = URL.createObjectURL(file);
  try {
    const image = new Image();
    image.decoding = "async";
    image.src = url;
    await image.decode();
    const canvas = document.createElement("canvas");
    canvas.width = limits.frameWidth;
    canvas.height = limits.frameHeight;
    const context = canvas.getContext("2d", { willReadFrequently: true });
    if (!context) throw new Error("无法读取载体图片。");
    context.fillStyle = "#202a31";
    context.fillRect(0, 0, canvas.width, canvas.height);
    const scale = Math.max(canvas.width / image.naturalWidth, canvas.height / image.naturalHeight);
    const drawWidth = image.naturalWidth * scale;
    const drawHeight = image.naturalHeight * scale;
    context.drawImage(
      image,
      (canvas.width - drawWidth) / 2,
      (canvas.height - drawHeight) / 2,
      drawWidth,
      drawHeight,
    );
    return context.getImageData(0, 0, canvas.width, canvas.height).data;
  } finally {
    URL.revokeObjectURL(url);
  }
}

function setReceiveSource(blob, name) {
  receiveSource = blob;
  $("#cipher-file-name").textContent = name;
  $("#cipher-file-size").textContent = formatBytes(blob.size);
  if (receiveSourceUrl) URL.revokeObjectURL(receiveSourceUrl);
  receiveSourceUrl = URL.createObjectURL(blob);
  const preview = $("#receive-preview");
  preview.src = receiveSourceUrl;
  preview.hidden = false;
  $("#receive-placeholder").hidden = true;
  $("#receive-preview-stage").classList.remove("empty");
  $("#decoded-result").hidden = true;
}

function wireDropControl(control, input, accept) {
  for (const eventName of ["dragenter", "dragover"]) {
    control.addEventListener(eventName, (event) => {
      event.preventDefault();
      control.classList.add("is-dragging");
    });
  }
  for (const eventName of ["dragleave", "drop"]) {
    control.addEventListener(eventName, (event) => {
      event.preventDefault();
      control.classList.remove("is-dragging");
    });
  }
  control.addEventListener("drop", (event) => {
    const file = event.dataTransfer?.files?.[0];
    if (!file || (accept && !accept(file))) return;
    const transfer = new DataTransfer();
    transfer.items.add(file);
    input.files = transfer.files;
    input.dispatchEvent(new Event("change", { bubbles: true }));
  });
}

modeTabs.forEach((tab) => tab.addEventListener("click", () => switchMode(tab.dataset.mode)));
document.querySelectorAll('input[name="content-kind"]').forEach((input) => {
  input.addEventListener("change", updateContentKind);
});

secretText.addEventListener("input", () => {
  $("#text-size").textContent = formatBytes(encoder.encode(secretText.value).length);
});

secretFile.addEventListener("change", () => updateSecretFile(secretFile.files?.[0]));
carrierFile.addEventListener("change", () => updateCarrierFile(carrierFile.files?.[0]));
cipherFile.addEventListener("change", () => {
  const file = cipherFile.files?.[0];
  if (file) setReceiveSource(file, file.name);
});

wireDropControl($("#secret-file-drop"), secretFile);
wireDropControl($("#carrier-file-drop"), carrierFile, (file) => file.type.startsWith("image/"));
wireDropControl(
  $("#cipher-file-drop"),
  cipherFile,
  (file) => file.type === "image/gif" || file.name.toLowerCase().endsWith(".gif"),
);

$("#generate-password").addEventListener("click", () => {
  sendPassword.value = createRandomPassword();
  sendPassword.type = "text";
  $("#show-send-password").checked = true;
  sendPassword.focus();
});

$("#show-send-password").addEventListener("change", (event) => {
  sendPassword.type = event.target.checked ? "text" : "password";
});
$("#show-receive-password").addEventListener("change", (event) => {
  receivePassword.type = event.target.checked ? "text" : "password";
});

sendForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const kind = document.querySelector('input[name="content-kind"]:checked').value;
  let content;
  try {
    if (kind === "text") {
      const data = encoder.encode(secretText.value);
      content = { kind: "text", data };
    } else {
      const file = secretFile.files?.[0];
      if (!file) throw new Error("请先选择文件。");
      if (file.size > limits.maxInputBytes) throw new Error("第一版最多加密 1 MB 文件。");
      content = {
        kind: "file",
        name: file.name,
        mime: file.type,
        data: new Uint8Array(await file.arrayBuffer()),
      };
    }
    const coverPixels = await readCarrierPixels(carrierFile.files?.[0]);
    setBusy(sendButton, true, "正在生成…", "生成加密 GIF");
    $("#send-result").hidden = true;
    const result = await createCipherGif(content, sendPassword.value, {
      coverPixels,
      onProgress: (message, fraction) => updateStatus("send", message, fraction),
    });
    generatedGif = result.blob;
    if (generatedGifUrl) URL.revokeObjectURL(generatedGifUrl);
    generatedGifUrl = URL.createObjectURL(result.blob);
    const preview = $("#send-preview");
    preview.src = generatedGifUrl;
    preview.hidden = false;
    $("#send-placeholder").hidden = true;
    $("#send-preview-stage").classList.remove("empty");
    $("#gif-size").textContent = formatBytes(result.blob.size);
    $("#gif-frames").textContent = `${result.frameCount} 帧 / ${result.dataFrameCount} 个分片`;
    $("#gif-message-id").textContent = result.messageId;
    const download = $("#download-gif");
    download.href = generatedGifUrl;
    download.download = `cipher-${result.messageId}.gif`;
    $("#send-result").hidden = false;
  } catch (error) {
    updateStatus("send", error instanceof Error ? error.message : String(error), 0, true);
  } finally {
    setBusy(sendButton, false, "正在生成…", "生成加密 GIF");
  }
});

$("#try-decrypt").addEventListener("click", () => {
  if (!generatedGif) return;
  setReceiveSource(generatedGif, $("#download-gif").download);
  receivePassword.value = sendPassword.value;
  switchMode("receive");
});

$("#save-gif").addEventListener("click", async () => {
  if (!generatedGif) return;
  const filename = $("#download-gif").download || "cipher.gif";
  const file = new File([generatedGif], filename, { type: "image/gif" });
  try {
    if (navigator.share && navigator.canShare?.({ files: [file] })) {
      await navigator.share({ title: "Cipher GIF", files: [file] });
      updateStatus("send", "已打开系统分享面板，可选择保存到照片。", 1);
    } else {
      $("#download-gif").click();
      updateStatus("send", "GIF 已下载，可从文件中保存到照片。", 1);
    }
  } catch (error) {
    if (error instanceof DOMException && error.name === "AbortError") return;
    updateStatus("send", error instanceof Error ? error.message : String(error), 0, true);
  }
});

receiveForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (!receiveSource) {
    updateStatus("receive", "请先选择 Cipher GIF。", 0, true);
    return;
  }
  setBusy(decryptButton, true, "正在解密…", "解密并验证");
  $("#decoded-result").hidden = true;
  if (decodedFileUrl) {
    URL.revokeObjectURL(decodedFileUrl);
    decodedFileUrl = "";
  }
  try {
    const bytes = new Uint8Array(await receiveSource.arrayBuffer());
    const result = await decodeCipherGif(bytes, receivePassword.value, (message, fraction) => {
      updateStatus("receive", message, fraction);
    });
    const textResult = $("#decoded-text");
    const fileResult = $("#decoded-file");
    const copyButton = $("#copy-text");
    const downloadFile = $("#download-file");
    textResult.hidden = true;
    fileResult.hidden = true;
    copyButton.hidden = true;
    downloadFile.hidden = true;

    if (result.kind === "text") {
      const text = decoder.decode(result.data);
      textResult.textContent = text;
      textResult.hidden = false;
      copyButton.hidden = false;
      copyButton.dataset.text = text;
    } else {
      $("#decoded-file-name").textContent = result.name;
      $("#decoded-file-meta").textContent = `${formatBytes(result.data.length)} · ${result.mime}`;
      fileResult.hidden = false;
      decodedFileUrl = URL.createObjectURL(new Blob([result.data], { type: result.mime }));
      downloadFile.href = decodedFileUrl;
      downloadFile.download = result.name;
      downloadFile.hidden = false;
    }
    $("#decoded-result").hidden = false;
  } catch (error) {
    updateStatus("receive", error instanceof Error ? error.message : String(error), 0, true);
  } finally {
    setBusy(decryptButton, false, "正在解密…", "解密并验证");
  }
});

$("#copy-text").addEventListener("click", async (event) => {
  const button = event.currentTarget;
  await navigator.clipboard.writeText(button.dataset.text || "");
  button.textContent = "已复制";
  setTimeout(() => {
    button.textContent = "复制文字";
  }, 1600);
});

updateContentKind();
