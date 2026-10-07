# 个人主页更新指南

网站：https://tanboning1118.github.io/

## 日常更新

1. 打开 `assets/home/content.js`，同时更新 `en`（英文）和 `zh`（中文）中的对应内容。
2. `projects` 管理项目，`experience` 管理实习/工作，`education` 管理教育经历，`awards` 管理荣誉；`email` 管理公开邮箱；`updated` 填本次更新日期。
3. 主页与 `cv.html` 共用这些内容。修改后，在线简历会同步更新。
4. 下载用 PDF 是静态快照：分别打开 `cv.html?lang=en` 与 `cv.html?lang=zh`，点击「打印 / 另存为 PDF」，纸张选 A4，关闭浏览器页眉页脚，保存并替换 `assets/home/Boning-Tan-CV-en.pdf` 和 `assets/home/Boning-Tan-CV-zh.pdf`。
5. 将修改提交到 GitHub 的 `main` 分支，等待 Pages 发布完成，再访问网站检查。

## 设计与结构

- `index.html`：现代个人主页结构和学术论文信息（新增论文时在这里添加，并同步 `assets/home/app.js` 的简历论文部分）。
- `assets/home/style.css`：主页配色、排版、手机适配。
- `assets/home/content.js`：中英文个人内容。
- `assets/home/app.js`：语言切换、项目详情、在线简历渲染。
- `cv.html` 与 `assets/home/cv.css`：简历页面与打印样式。
- `assets/home/favicon.svg`：网站图标。

网页不依赖第三方字体或前端库。默认英文，可切换中文并记住语言选择；分享 `?lang=zh` 链接可直接打开中文。

## 内容维护原则

只写已确认的学历、经历、论文和荣誉。研究兴趣与已完成成果分开；更新实习时注明起止时间。公开简历不包含证件号、学号、住址、手机号或证明原件。项目图形是示意图。

历史 `_posts`、`_tabs`、`cipher` 等内容继续保留。首页为独立静态 HTML，可由 GitHub Pages 直接发布，也能被现有 Jekyll 构建原样复制。
