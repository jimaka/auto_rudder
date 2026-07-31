#!/bin/bash
# gen_maneuver_matrix.sh — 生成船舶所有工况的机动效果视频矩阵
#   维度1: 船参数（typical K=0.2/T=8、fast K=0.5/T=3、slow K=0.1/T=20）
#   维度2: 机动形态（williamson/uturn/zigzag/circles/cloverleaf/search）
#   维度3: 干扰（无干扰 / 带 风+浪+噪声 干扰）
# 共 3 × 6 × 2 = 36 个工况视频，输出到 out/matrix/。
set -e
cd "$(dirname "$0")/.."
mkdir -p out/matrix

SIM=./build/sim/ar_sim
ANIM="python3 tools/animate_sim.py"
export ANIM_SIM_AGG=1

# 船参数：name|K|T
SHIPS=(
  "typical|0.20|8.00"
  "fast|0.50|3.00"
  "slow|0.10|20.00"
)
MANEUVERS=(williamson uturn zigzag circles cloverleaf search)

# 每个工况的总时长（秒）— 慢船/长机动需要更长（按实测序列完成时间标定）
duration_for() {
  case "$1:$2" in
    slow:cloverleaf)    echo 760 ;;  # 修正几何后实测约 744s 才完成
    slow:search)        echo 540 ;;  # 实测约 520s 才完成
    typical:cloverleaf) echo 480 ;;  # 实测约 398s 才完成
    slow:*)             echo 420 ;;
    typical:*)          echo 300 ;;
    *)                  echo 240 ;;
  esac
}

count=0
total=$((${#SHIPS[@]} * ${#MANEUVERS[@]} * 2))
for ship in "${SHIPS[@]}"; do
  IFS='|' read -r sname K T <<< "$ship"
  for m in "${MANEUVERS[@]}"; do
    sec=$(duration_for "$sname" "$m")
    for dist_flag in "" "--dist"; do
      tag="${sname}_${m}"
      [ -n "$dist_flag" ] && tag="${tag}_dist"
      csv="out/matrix/${tag}.csv"
      mp4="out/matrix/${tag}.mp4"
      count=$((count + 1))
      if [ -f "$mp4" ]; then
        echo "[$count/$total] 跳过(已存在): $tag"
        continue
      fi
      echo "[$count/$total] 生成: $tag (K=$K T=$T ${dist_flag:-nodist} ${sec}s)"
      $SIM --maneuver "$m" --K "$K" --T "$T" --seconds "$sec" $dist_flag --out "$csv" >/dev/null
      $ANIM "$csv" -o "$mp4" --polar --fps 20 --speed 3.0 \
        --title "${m}  ${sname}船  ${dist_flag:+带干扰}" >/dev/null
      echo "    OK: $mp4"
    done
  done
done

echo "=== 全部完成: $total 个工况视频 ==="
ls -la out/matrix/*.mp4 | wc -l
du -sh out/matrix/
