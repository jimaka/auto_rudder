# ESC 减参方案配图

对应文档：[ESC 减参方案设计文档](../ESC减参方案设计文档.md)。

配套文档：[实施阶段计划](../ESC减参方案实施阶段计划.md) · [回退流程设计](../ESC减参方案回退流程设计.md)。回退流程图保存在对应 Markdown 内，可通过项目的 `md2pdf.py` 导出 PDF 阅读。

## 图片与数据来源

| 文件名前缀 | 内容 | 数据来源 |
|---|---|---|
| `01_architecture` | 现场设置、船型配置、单变量 ESC 与安全门 | 本次设计提案的概念架构 |
| `02_timing_and_hpf` | 默认/示例窗口与载波、高通同相系数 | `AutoTuner::Params` 默认值与文档给出的解析公式 |
| `03_single_variable_path` | 固定名义阻尼的 Kp/Kd 曲线 | 极点配置解析式；K̂=0.2 s⁻¹，T̂=8 s，ζ*=0.85 |
| `04_dither_envelope` | 中心检查与完整载波包络检查 | 人工选取 q 边界 ±0.30、a=0.08 的算术示意 |

每张图提供 PNG（Markdown 直接显示）与 SVG（矢量排版）两种格式。图形为程序绘制，不是生成式图像，也不是闭环仿真/实船数据；未使用仓库中的 CSV 结果文件。

## 重生成

依赖 Python 3、numpy、matplotlib，以及 Noto Sans CJK 等中文字体。在仓库根目录运行：

```bash
MPLBACKEND=Agg python3 docs/esc_parameter_reduction/generate_figures.py
```

输出位置固定为本目录下的 `figures/`，只覆盖本方案的 4 组 PNG/SVG。运行环境若不能写 matplotlib 默认缓存目录，可另设 `MPLCONFIGDIR` 指向可写的临时目录。

生成脚本只核对图中的基础公式，不替代控制算法测试。修改参数或语义时，应同步更新正文算例和图注，并检查中文字体及标注是否遮挡。
