#!/bin/bash
# gen_all_videos.sh — 生成所有功能仿真视频
set -e
cd "$(dirname "$0")/.."
mkdir -p out

SIM=./build/sim/ar_sim
ANIM="python3 tools/animate_sim.py"
export ANIM_SIM_AGG=1

gen() {
    local name="$1"; shift
    local csv="out/${name}.csv"
    local mp4="out/${name}.mp4"
    if [ ! -f "$csv" ]; then
        echo ">>> 跑场景: $name"
        $SIM "$@" --out "$csv" >/dev/null 2>&1
    fi
    if [ ! -f "$mp4" ]; then
        echo ">>> 生成视频: $name"
        $ANIM "$csv" -o "$mp4" --fps 25 --speed 2.0 --polar --title "$name"
    fi
    echo "    OK: $mp4"
}

# 阶跃（无极坐标）
gen step --scenario step_typical
rm -f out/step.mp4
ANIM_SIM_AGG=1 python3 tools/animate_sim.py out/step.csv -o out/step.mp4 --fps 25 --speed 2.0 --title "30° 阶跃响应 (step_typical)"

# RLS 辨识
gen ident --ident
# ESC 长期
gen esc --esc

# 六机动形态
gen williamson --maneuver williamson
gen uturn --maneuver uturn
gen zigzag --maneuver zigzag
gen circles --maneuver circles
gen cloverleaf --maneuver cloverleaf
gen search --maneuver search

# 全验证矩阵（4 个 case）
$SIM --validate --out out/validate.csv >/dev/null 2>&1
for f in out/validate_*.csv; do
    name=$(basename "$f" .csv)
    mp4="out/${name}.mp4"
    if [ ! -f "$mp4" ]; then
        echo ">>> 生成视频: $name"
        $ANIM "$f" -o "$mp4" --fps 25 --speed 2.0 --title "$name"
    fi
done

echo "=== 全部完成 ==="
ls -la out/*.mp4
