# 仿真可视化工具（tools/）

`plot_sim.py` 读取 `ar_sim` 导出的 CSV，生成多子图分析图与机动极坐标轨迹图。
`animate_sim.py` 读取 CSV，生成时间演化的动画视频（MP4/GIF）。

## 依赖

系统已安装：`python3-matplotlib`、`python3-numpy`、`ffmpeg`。

```bash
# 若缺失，可一键安装
sudo apt-get install -y python3-matplotlib python3-numpy ffmpeg
```

## 一键流程：跑场景 → 导出 CSV → 生成图

```bash
cd control
mkdir -p out

# 1) 30° 阶跃（典型船参）
./build/sim/ar_sim --scenario step_typical --out out/step.csv
python3 tools/plot_sim.py out/step.csv -o out/step.png --metrics

# 2) RLS 闭环辨识
./build/sim/ar_sim --ident --out out/ident.csv
python3 tools/plot_sim.py out/ident.csv -o out/ident.png

# 3) ESC 长期
./build/sim/ar_sim --esc --out out/esc.csv
python3 tools/plot_sim.py out/esc.csv -o out/esc.png

# 4) 六机动形态（带极坐标轨迹）
./build/sim/ar_sim --maneuver williamson --out out/will.csv
python3 tools/plot_sim.py out/will.csv -o out/will.png --polar

# 5) 全验证矩阵（自动按 case 分文件导出）
./build/sim/ar_sim --validate --out out/validate.csv
python3 tools/plot_sim.py out/validate_*.csv --grid -o out/validate.png --metrics

# 6) 三船参阶跃叠加对比
./build/sim/ar_sim --all --out out/all.csv
python3 tools/plot_sim.py out/all_step_typical.csv out/all_step_fast.csv out/all_step_slow.csv \
    -o out/compare.png --metrics
```

## 子图说明（多子图模式）

| 子图 | 内容 |
|---|---|
| 1 | 航向 ψ 与参考 ψ_ref（°，时间） |
| 2 | 舵指令 δ_cmd 与实际 δ_act（含 ±35° 限幅虚线） |
| 3 | 转首率 r（°/s） |
| 4 | 航向误差 e（°） |
| 5 | 增益 Kp / Kd（双 y 轴） |

模式切换处用绿色点划竖线标注。

## 极坐标模式（`--polar`）

以航向 ψ 为极角、累计转首角为半径，绘制机动轨迹。零度朝北、顺时针，符合航海习惯。
起点（绿）、终点（红）单独标注。适合 williamson / uturn / circles / cloverleaf / search 等机动形态。

## 命令行选项

```
csv                   一个或多个 CSV 路径
-o, --out PATH        输出图片（PNG/PDF/SVG）；不指定则弹窗
--polar               追加机动极坐标图（单文件时输出 <stem>_polar.<ext>）
--no-multipane        不画多子图（仅 --polar 时有用）
--grid                多文件网格平铺
--metrics             打印指标表（超调/调节时间/稳态1σ/操舵频次）
--title TEXT          图总标题
```

## 输入输出环境变量

- `PLOT_SIM_AGG=1`：强制使用 Agg 后端（无 GUI 环境，仅保存图片）。

# 动画视频工具（animate_sim.py）

`animate_sim.py` 读取 CSV，生成时间演化的动画视频。左半部分为多子图（航向/参考、舵角、转首率、误差、增益），右半部分可选机动极坐标轨迹。

## 一键生成所有功能仿真视频

```bash
bash tools/gen_all_videos.sh
```

该脚本会自动跑所有场景（step/ident/esc/六机动/validate），导出 CSV 并生成对应 MP4 到 `out/`。

## 单独生成

```bash
# 阶跃
python3 tools/animate_sim.py out/step.csv -o out/step.mp4 --fps 25 --speed 2.0

# 机动 + 极坐标
python3 tools/animate_sim.py out/will.csv -o out/will.mp4 --polar --fps 25 --speed 2.0

# 无 ffmpeg 时用 GIF
python3 tools/animate_sim.py out/step.csv -o out/step.gif --fps 20
```

## 选项

```
csv                   CSV 路径
-o, --out PATH        输出视频（.mp4 需 ffmpeg，.gif 无需）
--fps N               帧率（默认 30）
--speed X             回放速度倍数（>1 加速，默认 1.0）
--polar               右侧追加机动极坐标轨迹
--title TEXT          视频标题
```

## 输入输出环境变量

- `ANIM_SIM_AGG=1`：强制使用 Agg 后端。

## 输出视频清单

| 视频 | 内容 |
|---|---|
| step.mp4 | 30° 阶跃响应（航向/舵角/转首率/误差/增益） |
| ident.mp4 | RLS 闭环辨识（Z 形激励 + 极坐标） |
| esc.mp4 | ESC 长期增益整定 |
| williamson.mp4 | Williamson 机动（极坐标） |
| uturn.mp4 | U 形回转（极坐标） |
| zigzag.mp4 | Z 形机动（极坐标） |
| circles.mp4 | 圆形回转（极坐标） |
| cloverleaf.mp4 | 三叶草机动（极坐标） |
| search.mp4 | 搜索机动（极坐标） |
| validate_*.mp4 | 全验证矩阵 4 个 case |

# 全工况机动视频矩阵（gen_maneuver_matrix.sh）

`gen_maneuver_matrix.sh` 生成船舶所有工况的机动效果视频矩阵，输出到 `out/matrix/`：

- **维度1 船参数**：typical (K=0.2/T=8)、fast (K=0.5/T=3)、slow (K=0.1/T=20)
- **维度2 机动形态**：williamson / uturn / zigzag / circles / cloverleaf / search
- **维度3 干扰**：无干扰 / 带风+浪+噪声干扰

共 3 × 6 × 2 = **36 个工况视频**，命名格式：`<船型>_<机动>[_dist].mp4`，例如 `typical_williamson_dist.mp4`。

```bash
bash tools/gen_maneuver_matrix.sh
```

每个视频含多子图（航向/舵角/转首率/误差/增益）+ 极坐标机动轨迹，标题标注船型与干扰状态。

