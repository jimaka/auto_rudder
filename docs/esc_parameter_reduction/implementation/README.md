# ESC 减参实施计划：图文产物

主文档：[ESC 减参方案实施阶段计划](../../ESC减参方案实施阶段计划.md)。  
离线阅读：[HTML 版](ESC减参实施计划.html)，含目录导航和打印样式。

## 六张配图

| 文件名前缀 | 图示内容 | 数据性质 |
|---|---|---|
| `01_stage_roadmap` | P0/A0/A1/B/C/D1/D2/E 的依赖与保留路径 | 拟议实施路线，无已完成状态 |
| `02_module_boundaries` | 配置推导、AutoTuner、Core、外围集成职责 | 基于当前架构的改造提议 |
| `03_window_contract` | 冻结、整周期、观测相位和结算 | 单位正弦的解析时间示意，不是仿真结果 |
| `04_comparison_groups` | REF/B0/B1/B2/B3 对照关系 | 试验设计，不包含性能成绩 |
| `05_acceptance_gates` | 阶段门与必交证据 | 计划中的待实施/待评审门禁 |
| `06_configuration_transaction` | 运行计划生效与 NVM 保存/恢复 | 拟议事务，不是现成 BSP 实现 |

每张图提供 `figures/` 下的 PNG 与 SVG。工作量以主文档中的条件性人日估算为准；图中不表示日历进度，也不使用历史 CSV 绘制虚构的收敛曲线。

## 生成与校验

在仓库根目录运行：

```bash
python3 docs/esc_parameter_reduction/implementation/build_document.py
```

依赖 Python、NumPy、Matplotlib、Noto Sans CJK 等中文字体、`markdown-it-py` 和 Pillow。脚本复用上一级 `generate_figures.py` 的绘图辅助函数，重定向输出到本目录，不执行控制算法，也不覆盖原设计方案的四组配图。

输出包括 6 组 PNG/SVG、`ESC减参实施计划.html`、`validation.json`。后者记录文档链接/图像/阶段结构检查，不是单元测试、HIL 或海试结果。修改正文任务和阶段时，应同步审阅图示，不能假设绘图脚本会理解任意代码变更。
