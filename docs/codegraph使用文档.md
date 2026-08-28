# CodeGraph 使用文档

> 工具：CodeGraph v1.5.0（MIT，GitHub: `colbymchenry/codegraph`）
> 集成对象：本机 Codex CLI 0.146.1（MCP + `~/.codex/AGENTS.md`）
> 文档日期：2026-08-07

## 1. 这是什么

CodeGraph 是一个**预索引代码知识图谱**工具：启动时把项目代码解析成符号图（函数、类、方法、调用关系、动态派发等），之后 agent 用一次查询就能拿到"相关符号的原文 + 调用链"，不必反复 grep / 逐个读文件，从而**省 token、省工具调用**。

核心特性：

| 特性 | 说明 |
|---|---|
| 100% 本地 | 索引与查询都在本机，不发送代码 |
| 自动同步 | 文件变化后自动增量更新（也可手动 `codegraph sync`） |
| 免 Node | 独立包自带运行时，Linux/macOS/Windows 直接装 |
| 多 agent | 官方支持 Codex CLI、Claude Code、Cursor、Gemini 等 |
| 双层接口 | MCP 工具（agent 会话内）+ CLI（shell 直接用） |

需要明确：CodeGraph **不是 SKILL.md 形式的 Codex skill**，而是以 MCP 服务器方式接入 Codex；仓库内 `.claude/skills/` 下的 skill 只服务于 Claude Code。

## 2. 安装与接入

### 2.1 安装 CLI（免 Node 独立包）

```bash
curl -fsSL https://raw.githubusercontent.com/colbymchenry/codegraph/main/install.sh | sh
```

安装位置：`~/.codegraph/`（运行包），可执行文件链接到 `~/.local/bin/codegraph`。验证：

```bash
codegraph --version   # 例如 1.5.0
```

有 Node 20~24 时也可用 npm：`npm i -g @colbymchenry/codegraph`。

### 2.2 接入 Codex（MCP）

```bash
codegraph install -l global -y
```

对 Codex CLI 做两件事：

1. 在 `~/.codex/config.toml` 写入：

```toml
[mcp_servers.codegraph]
command = "codegraph"
args = ["serve", "--mcp"]
```

2. 创建/更新 `~/.codex/AGENTS.md`，加入 CodeGraph 使用提示（有 `.codegraph/` 的项目优先使用它）。

**必须重启 Codex 会话后 MCP 工具才生效。** 该命令还会自动检测并接入 Claude Code、Cursor、Gemini、Antigravity 等已安装的 agent（只写对应配置文件）；只想保留 Codex 时可先 `codegraph uninstall -l global -y` 再按需重装。

### 2.3 升级与卸载

```bash
codegraph upgrade              # 升级到最新版
codegraph upgrade --check      # 只看是否有新版
codegraph uninstall -l global -y --keep-cli   # 只摘除 agent 配置，保留 CLI
```

彻底卸载 CLI：`curl -fsSL https://raw.githubusercontent.com/colbymchenry/codegraph/main/install.sh | sh -s -- --uninstall`。

## 3. 项目初始化与索引维护

每个项目只需初始化一次：

```bash
cd <项目目录>
codegraph init
```

在项目根目录生成 `.codegraph/`（本地索引，建议加入 `.gitignore`），并立即建立全量索引。常用维护命令：

| 命令 | 作用 |
|---|---|
| `codegraph init [path]` | 初始化并建全量索引 |
| `codegraph index [path]` | 从零重建索引（等价于重新 init） |
| `codegraph sync [path]` | 增量同步自上次索引以来的变更（可挂 git hook） |
| `codegraph status [path]` | 查看索引状态与统计（`-j` 输出 JSON） |
| `codegraph daemons` | 管理后台守护进程（自动同步服务） |

本仓库（auto_rudder）现状：

```text
Files: 27 | Nodes: 518 | Edges: 1,640 | DB: 1.87 MB
Node 构成：function 166、import 117、method 114、variable 35、struct 28 …
语言：cpp 21、python 6
```

## 4. 常用命令

### 4.1 符号搜索

```bash
codegraph query AutoTuner          # 搜符号，默认 10 条
codegraph query --kind class Auto  # 按类型过滤（function/class/method/…）
codegraph query --json tick        # JSON 输出，便于脚本处理
```

### 4.2 区域探索（最常用）

```bash
codegraph explore "AutoTuner tick 调用链"
codegraph explore "辨识器如何喂给调参器"
```

一次返回相关符号的原文 + 调用路径（与 MCP 工具 `codegraph_explore` 同一输出），适合回答"XX 是怎么工作的 / 改动会影响谁"。

### 4.3 单符号 / 单文件视图

```bash
codegraph node AutoTuner::tick      # 符号源码 + 调用者/被调用者
codegraph node -f control/include/auto_tuner.hpp   # 文件行号视图 + 依赖方
codegraph node -f control/include/auto_tuner.hpp --offset 150 --limit 40
```

### 4.4 项目文件结构

```bash
codegraph files                              # 树形文件结构
codegraph files --pattern '*.hpp' --flat     # 按 glob 过滤
codegraph files --filter control --max-depth 3
```

## 5. MCP 工具（Codex 会话内）

重启 Codex 后，会话内可用以下 MCP 工具（名称以实际会话提示为准）：

| 工具 | 用途 |
|---|---|
| `codegraph_explore` | 一次查询：相关符号原文 + 调用链（替代多轮 grep/读文件） |
| `codegraph_node` | 单符号源码 + 调用者/被调用者，或文件行号视图 |
| `codegraph_search` | 符号搜索 |
| `codegraph_callers` / `codegraph_callees` | 只查调用者 / 被调用者 |
| `codegraph_context` | 上下文聚合（变更影响面） |
| `codegraph_impact` | 影响分析（改一个符号波及哪些代码） |
| `codegraph_files` | 项目文件结构 |
| `codegraph_status` | 索引状态 |

使用约定（已写入 `~/.codex/AGENTS.md`）：**只有项目根存在 `.codegraph/` 时才使用 CodeGraph**；未索引的项目直接跳过，索引与否由用户决定。

## 6. 典型工作流

1. **理解一段代码**：`codegraph explore "自动调参 tick 内部顺序"` → 拿到符号原文与调用链，再针对性地读 1~2 个文件。
2. **改代码前评估影响**：Codex 会话内用 `codegraph_impact` / `codegraph_callers <符号>` 看谁依赖它；shell 用 `codegraph node <符号>` 看实现与调用者/被调用者。
3. **改完代码**：MCP 服务运行期间文件监听会自动增量同步；也可手动 `codegraph sync`，再 `codegraph status` 确认索引最新，避免 agent 拿到旧图。
4. **跨文件/动态派发**：普通 grep 追不动的地方（如机动序列多态、注册表分发），用 `codegraph_explore` 一次拿全路径。

## 7. 配置与遥测

- 遥测：默认收集匿名使用统计（不含代码/路径/名称）；查看或关闭：

```bash
codegraph telemetry status
codegraph telemetry off
```

- MCP 服务参数：需要关闭文件监听时（如 WSL2 慢盘），把 `~/.codex/config.toml` 中 args 改为 `["serve", "--mcp", "--no-watch"]`，再重启 Codex。
- 项目路径：MCP 模式默认取客户端传入的项目根（rootUri）；shell 命令可用 `-p <path>` 指定项目。

## 8. 常见问题

| 现象 | 处理 |
|---|---|
| Codex 会话里没有 codegraph 工具 | 确认 `~/.codex/config.toml` 有 `[mcp_servers.codegraph]` 后**重启 Codex** |
| 仓库没有 `.codegraph/` | 先 `codegraph init`；未索引时工具/AGENTS.md 会跳过 |
| 查询结果旧 | `codegraph sync` 或 `codegraph index` 重建，再 `codegraph status` 确认 |
| 大仓/慢盘同步卡顿 | `serve` 加 `--no-watch`，改用 git hook 或手动 `sync` |
| 想换版本 | `codegraph upgrade <版本>` 固定版本，`--check` 先看更新 |
| `.codegraph/` 进了 git | 在项目 `.gitignore` 加一行 `.codegraph/`（本地索引无需入库） |

## 9. 参考

- 官方仓库：<https://github.com/colbymchenry/codegraph>
- 文档站：<https://colbymchenry.github.io/codegraph/>
- 本地安装：`~/.codegraph/versions/v1.5.0`，可执行文件 `~/.local/bin/codegraph`
- 本机接线文件：`~/.codex/config.toml`、`~/.codex/AGENTS.md`
