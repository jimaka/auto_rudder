# Issue tracker：本地 Markdown

本仓库的 issue 与 spec（你可能把 spec 称作 PRD）以 markdown 文件的形式存放在 `.scratch/` 中。

## 约定

- 每个 feature 一个目录：`.scratch/<feature-slug>/`
- spec 是 `.scratch/<feature-slug>/spec.md`
- 实现类 issue 每个工单一个文件，位于 `.scratch/<feature-slug>/issues/<NN>-<slug>.md`，从 `01` 开始编号——绝不使用单个合并的工单文件
- 分诊状态记录为每个 issue 文件顶部附近的一行 `Status:`（角色字符串见 `triage-labels.md`）
- 评论与对话历史追加到文件底部的 `## Comments` 标题之下

## 当 skill 说"发布到 issue tracker"时

在 `.scratch/<feature-slug>/` 下创建一个新文件（如需要则先创建目录）。

## 当 skill 说"获取相关工单"时

读取所引用路径上的文件。用户通常会直接传入路径或 issue 编号。

## Wayfinding 操作

供 `/wayfinder` 使用。**map** 是一个文件，每个工单对应一个**子**文件。

- **Map**：`.scratch/<effort>/map.md` —— 承载 Notes / Decisions-so-far / Fog 正文。
- **子工单**：`.scratch/<effort>/issues/NN-<slug>.md`，从 `01` 开始编号，正文中写问题。一行 `Type:` 记录工单类型（`research`/`prototype`/`grilling`/`task`）；一行 `Status:` 记录 `claimed`/`resolved`。
- **阻塞关系**：顶部附近的一行 `Blocked by: NN, NN`。当其列出的所有文件都为 `resolved` 时，工单解除阻塞。
- **边界（Frontier）**：扫描 `.scratch/<effort>/issues/` 中未关闭、未阻塞、未认领的文件；编号最小者胜出。
- **认领（Claim）**：设为 `Status: claimed` 并在开始任何工作前保存。
- **解决（Resolve）**：在 `## Answer` 标题下追加答案，设为 `Status: resolved`，然后向 `map.md` 中 map 的 Decisions-so-far 追加一条上下文指针（gist + 链接）。
