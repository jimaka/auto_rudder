# 优化报告 V2 附件

对应[工程优化分析报告](../工程优化分析报告.md)，审查日期 2026-09-09，基线提交 `833d3c57e9066aef0037462dc57c634334db07a3` 加工作区的 GainSchedule 与相关测试改动。

主机环境：GCC 15.2.0、CMake 4.2.3、Python 3.14.4。对象大小仅代表此主机 ABI；没有目标 STM32 内存/时序结果。

- `validation_review_v2.png`：当前 `--validate` 四场景重新生成的航向曲线，随报告保留。
- `control_review_probe.cpp`：PD 量化/微分、ESC 包络与生命周期、RLS 坏帧时间的独立复现。
- `storage_review_probe.cpp`：参数容量、范围检查、失败保存与主机对象大小的独立复现。

两个探针只打印当前行为，退出 0 不表示被测算法正确。用于说明报告证据，不加入生产构建或以这些错误输出作为期望值的回归测试。

## 复现

在仓库根目录执行，输出使用新临时目录：

```bash
review_dir=$(mktemp -d /tmp/auto-rudder-review-XXXXXX)
g++ -std=c++17 -O2 -I control/include docs/optimization_assets/control_review_probe.cpp -o "$review_dir/control_probe"
g++ -std=c++17 -O2 -I control/include docs/optimization_assets/storage_review_probe.cpp -o "$review_dir/storage_probe"
"$review_dir/control_probe"
"$review_dir/storage_probe"
```

当前关键输出：

```text
quant_rate dt=0.016 cmd=0.2 bound=0.16
quant_stall dt=0.004 after1000=0
quant_amplitude dMax=34.96 cmd=35
wrap_derivative cmd=-35 shortestDiffExpected=10
deadzone_resume cmd=17.938 continuousFilterExpected=1.40909090909
dither established=1 ... wn=0.500012499842 stability=0
osc_before_suspend=20 after_resume_changed_ref=20
rows=111 bytes=4076 save=1 load=1
rows=112 bytes=4112 save=1 load=0
invalid_schedule_validate=1
failed_save=0 version=5->6 timestamp=123->456 crc_changed=1
default_config_validate=0
```

## 重新生成曲线

使用上面创建的临时目录，逐条执行。当前 `--validate` 应返回 1（两工况未达标），CSV 仍会生成；在自动化脚本中应分别处理性能失败与进程/IO错误，不能用 `|| true` 笼统忽略全部错误。

```bash
control/build/sim/ar_sim --validate --out "$review_dir/validate.csv"

MPLBACKEND=Agg MPLCONFIGDIR="$review_dir/mpl" python3 control/tools/plot_sim.py \
  "$review_dir/validate_typical_nodist.csv" \
  "$review_dir/validate_typical_dist.csv" \
  "$review_dir/validate_fast_nodist.csv" \
  "$review_dir/validate_slow_nodist.csv" \
  --grid --title '二次复审：四场景闭环响应（2026-09-09）' \
  -o "$review_dir/validation_review_v2.png"
```

图片只展示航向；舵机活动、频次口径与 ESC 改善应结合报告的独立判据验证。
