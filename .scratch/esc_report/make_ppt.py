#!/usr/bin/env python3
"""make_ppt.py — 生成《ESC 极值搜索自动调参方案汇报》PPT（16:9，19 页）

素材来源：.scratch/esc_report/math_derivation.md、code_facts.md、assets/esc_*.png
输出：ppt_output/ESC方案汇报.pptx
用法：.venv-ppt/bin/python .scratch/esc_report/make_ppt.py
"""
import os
from PIL import Image
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn
from lxml import etree

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ASSETS = os.path.join(ROOT, "assets")
OUT_DIR = os.path.join(ROOT, "ppt_output")
OUT = os.path.join(OUT_DIR, "ESC方案汇报.pptx")

# ---------- 设计规范：深蓝主色 + 白底 ----------
BLUE = "1F4E79"      # 主色深蓝
BLUE_D = "17375E"    # 更深
BLUE_L = "2E75B6"    # 亮蓝
ACCENT = "D9822B"    # 橙强调
TEAL = "2E86A1"
INK = "262626"       # 正文
MUTED = "595959"
FAINT = "7F7F7F"
BOX_BG = "EEF3FA"    # 浅蓝填充
BOX_LINE = "B8CCE4"
WARN_BG = "FDF3E7"   # 浅橙填充
WARN_LINE = "E0C090"
WHITE = "FFFFFF"
TITLE_SUB = "BBD2EA" # 标题栏右侧标签

FONT = "Microsoft YaHei"

EMU_W, EMU_H = 12192000, 6858000  # 16:9 精确尺寸
PAGE_W, PAGE_H = 13.3333, 7.5     # 英寸
MARGIN = 0.55
CONTENT_Y0 = 1.18
CONTENT_Y1 = 7.00

prs = Presentation()
prs.slide_width = Emu(EMU_W)
prs.slide_height = Emu(EMU_H)
BLANK = prs.slide_layouts[6]

TOTAL_PAGES = 19
_page_no = 0


def _set_font(run, name):
    """同时设置 latin / ea / cs 字体，保证中文与符号同族。"""
    run.font.name = name
    rPr = run._r.get_or_add_rPr()
    for tag in ("a:ea", "a:cs"):
        e = rPr.find(qn(tag))
        if e is None:
            e = etree.SubElement(rPr, qn(tag))
        e.set("typeface", name)


def rect(slide, x, y, w, h, fill=None, line=None, lw=0.75, rounded=False, radius=0.10):
    shp_type = MSO_SHAPE.ROUNDED_RECTANGLE if rounded else MSO_SHAPE.RECTANGLE
    shp = slide.shapes.add_shape(shp_type, Inches(x), Inches(y), Inches(w), Inches(h))
    if rounded:
        try:
            shp.adjustments[0] = radius
        except Exception:
            pass
    if fill is None:
        shp.fill.background()
    else:
        shp.fill.solid()
        shp.fill.fore_color.rgb = RGBColor.from_string(fill)
    if line is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = RGBColor.from_string(line)
        shp.line.width = Pt(lw)
    shp.shadow.inherit = False
    return shp


def tb(slide, x, y, w, h, anchor=MSO_ANCHOR.TOP, wrap=True):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = wrap
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.04)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    return tf


def para(tf, runs, first=False, align=PP_ALIGN.LEFT, sa=4, sb=0, ls=1.06):
    """runs: [(text, size, bold, color[, italic]), ...]"""
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_after = Pt(sa)
    p.space_before = Pt(sb)
    p.line_spacing = ls
    for item in runs:
        text, size, bold, color = item[0], item[1], item[2], item[3]
        italic = item[4] if len(item) > 4 else False
        r = p.add_run()
        r.text = text
        r.font.size = Pt(size)
        r.font.bold = bold
        r.font.italic = italic
        r.font.color.rgb = RGBColor.from_string(color)
        _set_font(r, FONT)
    return p


def pic(slide, name, x, y, w, h):
    """按原始纵横比缩放入盒、居中，不拉伸。"""
    path = os.path.join(ASSETS, name)
    with Image.open(path) as im:
        iw, ih = im.size
    scale = min(w / iw, h / ih)
    pw, ph = iw * scale, ih * scale
    px, py = x + (w - pw) / 2, y + (h - ph) / 2
    slide.shapes.add_picture(path, Inches(px), Inches(py), Inches(pw), Inches(ph))


def new_slide():
    return prs.slides.add_slide(BLANK)


def content_slide(title, tag):
    """统一版式：深蓝标题条 + 橙强调线 + 页脚。"""
    global _page_no
    _page_no += 1
    s = new_slide()
    rect(s, 0, 0, PAGE_W, 0.92, fill=BLUE)
    rect(s, 0, 0.92, PAGE_W, 0.045, fill=ACCENT)
    tf = tb(s, MARGIN, 0.08, 9.6, 0.76, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(title, 26, True, WHITE)], first=True, sa=0)
    tf = tb(s, 10.2, 0.18, 2.58, 0.56, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(tag, 14, False, TITLE_SUB)], first=True, align=PP_ALIGN.RIGHT, sa=0)
    # 页脚
    rect(s, MARGIN, 7.09, PAGE_W - 2 * MARGIN, 0.012, fill="D9D9D9")
    tf = tb(s, MARGIN, 7.13, 9.5, 0.32)
    para(tf, [("ESC 极值搜索自动调参方案汇报 · 船舶自动舵", 14, False, FAINT)],
         first=True, sa=0)
    tf = tb(s, 11.4, 7.13, 1.38, 0.32)
    para(tf, [(f"{_page_no} / {TOTAL_PAGES}", 14, False, FAINT)],
         first=True, align=PP_ALIGN.RIGHT, sa=0)
    return s


def heading(tf, text, first=False, sa=4, sb=6, size=17, color=BLUE):
    return para(tf, [(text, size, True, color)], first=first, sa=sa, sb=sb)


def bullet(tf, text, size=14, sa=4, color=INK, bold=False, mark="• "):
    return para(tf, [(mark, size, True, BLUE_L), (text, size, bold, color)], sa=sa, ls=1.1)


def formula_box(slide, x, y, w, h, lines, bg=BOX_BG, line=BOX_LINE):
    """lines: list of paragraphs, 每个为 runs 列表。"""
    rect(slide, x, y, w, h, fill=bg, line=line, rounded=True, radius=0.06)
    tf = tb(slide, x + 0.12, y + 0.06, w - 0.24, h - 0.12, anchor=MSO_ANCHOR.MIDDLE)
    for i, runs in enumerate(lines):
        para(tf, runs, first=(i == 0), sa=3, ls=1.08)
    return tf


# ============================================================
# S1 封面
# ============================================================
_page_no += 1
s = new_slide()
rect(s, 0, 0, PAGE_W, PAGE_H, fill=BLUE)
rect(s, 0, 6.62, PAGE_W, 0.88, fill=BLUE_D)
tf = tb(s, 1.0, 2.15, 11.3, 1.15, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("ESC 极值搜索自动调参方案汇报", 40, True, WHITE)], first=True, sa=0)
rect(s, 1.02, 3.42, 2.2, 0.06, fill=ACCENT)
tf = tb(s, 1.0, 3.62, 11.3, 0.6)
para(tf, [("船舶自动舵 PD 增益 Kp / Kd 在线微调 —— 原理 · 实现 · 安全 · 仿真验证", 20, False, "D7E3F2")],
     first=True, sa=0)
tf = tb(s, 1.0, 4.55, 11.3, 1.5)
para(tf, [("汇报依据：control/include/auto_tuner.hpp 实现 · 数学推导与代码事实（.scratch/esc_report/）", 14, False, "A9C3E0")], first=True, sa=5)
para(tf, [("仿真验证：ar_sim --tune（典型船 K=0.2 / T=8，18 个 60 s 评估窗口）", 14, False, "A9C3E0")], sa=5)
para(tf, [("2026-08-21", 14, False, "A9C3E0")], sa=0)
tf = tb(s, 1.0, 6.86, 11.3, 0.4)
para(tf, [("Extremum Seeking Control · 免模型在线寻优 · 三层调参架构", 14, False, "7FA3CC")],
     first=True, align=PP_ALIGN.CENTER, sa=0)

# ============================================================
# S2 目录
# ============================================================
s = content_slide("目录", "CONTENTS")
toc = [
    ("01", "背景与问题：为什么固定增益不够 · 三层调参架构"),
    ("02", "被控对象与闭环模型：Nomoto · 极点配置公式"),
    ("03", "代价函数设计：ITAE + 操舵惩罚"),
    ("04", "ESC 数学原理：摄动 → 解调 → 积分"),
    ("05", "双参数正交载波：Kp 用 sin / Kd 用 cos"),
    ("06", "时间尺度三分离与稳定性依据"),
    ("07", "信号链与实现流程：50 Hz 每拍 + 60 s 窗末结算"),
    ("08", "安全机制四道防线 · 调参生命周期"),
    ("09", "仿真验证：辨识 · 载波 · 增益轨迹 · 全链路"),
    ("10", "已知限制 · 改进方向 · 总结"),
]
for i, (num, text) in enumerate(toc):
    col, row = divmod(i, 5)
    x = MARGIN + col * 6.35
    y = 1.55 + row * 1.02
    rect(s, x, y, 0.62, 0.62, fill=BLUE, rounded=True, radius=0.18)
    tf = tb(s, x, y, 0.62, 0.62, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(num, 16, True, WHITE)], first=True, align=PP_ALIGN.CENTER, sa=0)
    tf = tb(s, x + 0.8, y - 0.05, 5.35, 0.95, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(text, 15, False, INK)], first=True, sa=0, ls=1.12)

# ============================================================
# S3 背景与问题
# ============================================================
s = content_slide("背景与问题：为什么需要在线调参", "01 · 背景")
tf = tb(s, MARGIN, 1.25, 5.75, 3.95)
heading(tf, "为什么固定增益不够", first=True)
bullet(tf, "Nomoto 参数 K / T 随船速、载况、海况缓变，固定增益无法始终匹配对象")
bullet(tf, "增益调度表只有 12 个分段名义值（速度 × 海况），颗粒度粗、依赖离线标定")
bullet(tf, "人工整定依赖经验、出海标定成本高，且无法在线跟踪参数漂移")
bullet(tf, "保向工况占航行时间绝大多数：增益微调空间直接对应燃油能耗与舵机磨损")
para(tf, [("指标牵引：", 14, True, ACCENT),
          ("航向保持优化约省 0.1–1% 燃油；操舵频次 ≤ 6 次/分", 14, False, INK)], sb=6, sa=0, ls=1.12)

layers = [
    ("L1  增益调度表（粗调）", "速度 × 海况 12 分段名义值 · gain_schedule.hpp", BOX_BG, BLUE),
    ("L2  极点配置（模型基线）", "RLS 辨识 Khat / That → Kp = ωn²That/Khat · auto_tuner.hpp:228-237", BOX_BG, BLUE),
    ("L3  ESC 在线微调（本方案）", "以代价 J 梯度寻优，每窗更新 ≤ ±10% · auto_tuner.hpp:395-426", WARN_BG, ACCENT),
]
for i, (t, d, bg, lc) in enumerate(layers):
    y = 1.30 + i * 1.32
    rect(s, 6.65, y, 6.13, 1.02, fill=bg, line=lc, rounded=True, radius=0.10)
    tf = tb(s, 6.85, y + 0.08, 5.75, 0.86, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t, 16, True, BLUE)], first=True, sa=2)
    para(tf, [(d, 14, False, MUTED)], sa=0)
    if i < 2:
        tf = tb(s, 6.65, y + 1.02, 6.13, 0.30, anchor=MSO_ANCHOR.MIDDLE)
        para(tf, [("▼", 14, True, BLUE_L)], first=True, align=PP_ALIGN.CENTER, sa=0)
rect(s, MARGIN, 5.42, 12.23, 1.50, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.08)
tf = tb(s, MARGIN + 0.18, 5.54, 11.9, 1.28, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("分工要点：", 15, True, BLUE),
          ("ESC 只做基线邻域内的局部微调 —— 平均化理论（Krstić–Wang, Automatica 2000）只保证局部收敛；"
           "全局粗调交给调度表与极点配置。三层架构正是这一理论分工的工程落地。", 15, False, INK)],
     first=True, sa=0, ls=1.15)

# ============================================================
# S4 被控对象与闭环模型
# ============================================================
s = content_slide("被控对象与闭环模型", "02 · 模型")
tf = tb(s, MARGIN, 1.22, 6.35, 5.7)
heading(tf, "一阶 Nomoto 模型", first=True, sb=2)
para(tf, [("T·(dr/dt) + r = K·δ  ⇒  G(s) = ψ(s)/δ(s) = K/[s·(T·s+1)]", 16, True, INK)], sa=3, ls=1.1)
para(tf, [("K 旋回性指数（1/s）、T 追随性指数（s）；随工况缓变，由 RLS 在线辨识（nomoto_identifier.hpp），只读快照送入 AutoTuner", 14, False, MUTED)], sa=6, ls=1.12)
heading(tf, "PD 控制律")
para(tf, [("δ = Kp·e + Kd·(de/dt)，e = ψr − ψ", 16, True, INK)], sa=3, ls=1.1)
para(tf, [("控制律实现 pd_controller.hpp:90；工程环节：0.3° 死区 · 微分低通 τ=0.2 s · 舵角 ±35° / 舵速 10°/s 限幅 · 0.1° 量化（:66-109）；保向工况下近似线性", 14, False, MUTED)], sa=6, ls=1.12)
heading(tf, "闭环特征多项式（对照标准二阶）")
para(tf, [("T·s² + (1 + K·Kd)·s + K·Kp = 0   ↔   s² + 2ζωn·s + ωn² = 0", 16, True, INK)], sa=3, ls=1.1)
para(tf, [("逐项对照：ωn² = K·Kp/T，2ζωn = (1 + K·Kd)/T", 14, False, MUTED)], sa=0, ls=1.12)

formula_box(s, 7.15, 1.22, 5.63, 1.72, [
    [("极点配置（全仓库唯一实现 auto_tuner.hpp:233-234）", 14, True, BLUE)],
    [("Kp = ωn²·That / Khat", 17, True, INK)],
    [("Kd = (2·ζ·ωn·That − 1) / Khat", 17, True, INK)],
    [("kdRaw < 0 时钳 0（:235）：负 Kd 反向放大振荡，阻尼由对象天然提供", 14, False, MUTED)],
])
formula_box(s, 7.15, 3.10, 5.63, 1.62, [
    [("稳定门反解（stabilityCheck :247-248）", 14, True, BLUE)],
    [("ωn' = √(Kp·Khat/That)", 15, False, INK)],
    [("ζ' = (1 + Kd·Khat) / (2·√(That·Kp·Khat))", 15, False, INK)],
    [("与极点配置正/反解严格互逆，配置与门限同口径", 14, False, MUTED)],
])
formula_box(s, 7.15, 4.88, 5.63, 2.00, [
    [("目标模态 ζ=0.85、ωn=0.20 rad/s 的物理意义", 14, True, BLUE)],
    [("超调 ≈0.6%（30° 阶跃仅 0.2°），ts ≈ 3/(ζωn) ≈ 17.6 s", 14, False, INK)],
    [("闭环带宽明显低于波浪遭遇频率（0.31~1.26 rad/s）—— 不打舵对抗波频摇艏，节能口径的正确选择", 14, False, INK)],
])

# ============================================================
# S5 代价函数设计
# ============================================================
s = content_slide("代价函数设计：误差 + 操舵代价", "03 · 代价")
formula_box(s, MARGIN, 1.22, 12.23, 1.30, [
    [("J = ∫[0,Tw] ( t·|e(t)| + wU·|δ(t)| + wDu·|dδ/dt| ) dt   ≈   Σ ( t_k·|e_k| + wU·|δ_k| + wDu·|Δδ_k| )·dt", 17, True, INK)],
    [("Tw = 60 s · N = 3000 拍 · 非重叠窗、结后清零 · computeJ :214-223 / accumulate :366-367", 14, False, MUTED)],
])
cards = [
    ("① t·|e| — ITAE 时间加权",
     "早期误差廉价、晚期昂贵：收敛慢被 τ² 惩罚；稳态静差贡献 ∝ Tw²/2，持续消差压力；"
     "对峰值不敏感，与“不超调优先”的 ζ=0.85 一致"),
    ("② wU·|δ| — 舵角惩罚",
     "舵诱导阻力 ∝ δ²，|δ| 是其线性代理（符号鲁棒、零开销）；直航保向平均舵角 ≈ 推力损失，"
     "对应 0.1–1% 燃油节能口径"),
    ("③ wDu·|dδ/dt| — 舵速惩罚",
     "操舵行程 ∫|dδ/dt|·dt ↔ 液压舵机功耗与磨损；抑制抖振（hunting），护舵机；"
     "对应操舵频次 ≤ 6 次/分指标"),
]
for i, (t, d) in enumerate(cards):
    x = MARGIN + i * 4.15
    rect(s, x, 2.72, 3.94, 2.52, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
    tf = tb(s, x + 0.16, 2.86, 3.62, 2.28)
    para(tf, [(t, 15, True, BLUE)], first=True, sa=5, ls=1.1)
    para(tf, [(d, 14, False, INK)], sa=0, ls=1.16)
rect(s, MARGIN, 5.46, 12.23, 1.46, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.08)
tf = tb(s, MARGIN + 0.18, 5.56, 11.9, 1.26, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("权重即旋钮：", 15, True, ACCENT),
          ("wU=0.01、wDu=0.005。调大操舵权重，ESC 自动收敛到更“懒”的增益 —— 节能口径由此进入调参目标；"
           "最优点略偏离纯 ITAE 最优点，属有意设计。", 15, False, INK)], first=True, sa=4, ls=1.15)
para(tf, [("工程细节：窗口样本存 RingBuffer<double,3000>，主循环零堆分配（:479-482）；ITAE 时钟每窗从 0 重启，各窗 J 同口径。", 14, False, MUTED)], sa=0, ls=1.12)

# ============================================================
# S6 ESC 数学原理（上）：摄动与梯度信息
# ============================================================
s = content_slide("ESC 数学原理（上）：摄动携带梯度信息", "04 · 原理")
tf = tb(s, MARGIN, 1.22, 5.60, 5.7)
heading(tf, "最优点附近的二次模型", first=True, sb=2)
para(tf, [("J(θ) ≈ J* + ½·J''·(θ − θ*)²，J'' > 0", 16, True, INK)], sa=3)
para(tf, [("当前点梯度 = J''·(θc − θ*)：与偏差成正比且带符号 —— 测出梯度即知方向与距离（θc 为去载波中心值）", 14, False, MUTED)], sa=6, ls=1.14)
heading(tf, "注入相对摄动（escDither :382-388）")
para(tf, [("θ(t) = θc·(1 + a·sinωt)，a = 5%", 16, True, INK)], sa=6)
heading(tf, "J(t) 谐波分解（系数写全）")
para(tf, [("J(t) = J0 + a·θc·(∂J/∂θ)·sinωt − ¼·J''·θc²a²·cos2ωt", 15, True, INK)], sa=4, ls=1.12)
bullet(tf, "基波幅值 ∝ 梯度，同相/反相携带符号 → 可被相关检测提取", size=14)
bullet(tf, "二倍频不含梯度信息 → 靠整周期平均消除", size=14)
bullet(tf, "直流 J0 比基波大几个数量级 → 必须先经 HPF 去除", size=14, sa=0)
pic(s, "esc_math_gradient.png", 6.35, 1.22, 6.45, 5.70)

# ============================================================
# S7 ESC 数学原理（下）：解调与平均
# ============================================================
s = content_slide("ESC 数学原理（下）：解调 · 平均 · 归一化", "04 · 原理")
formula_box(s, MARGIN, 1.18, 12.23, 1.02, [
    [("核心公式（escStep :399-405）：  ĝ = 2·Σ( y_k·sinφ_k ) / (a·N)  −→  θc·(∂J/∂θ)·ReH(jω)", 17, True, INK)],
    [("ωτH ≫ 1 时 ReH → 1，ĝ 恰为“J 对相对摄动的梯度”；系数 2 / a / N 三因子各有严格来历", 14, False, MUTED)],
])
tf = tb(s, MARGIN, 2.42, 5.85, 4.5)
steps = [
    ("① HPF 去直流（:369-371）", "y_k = αH·(y_{k-1} + c_k − c_{k-1})，αH = τH/(τH+dt)；去直流、留交流"),
    ("② 同相载波解调（:373）", "demod += y·sinφ；注入与解调共用同一 escPhase_，因果同相"),
    ("③ 窗口平均（整周期）", "均值(sin²ωt)=1/2，均值(cos2ωt·sinωt)=0，直流×sin 均值 0 —— 只留梯度直流"),
    ("④ 归一化 ×2/(aN)", "2 抵消 sin² 均值 1/2；a 还原单位摄动；N 把求和转均值"),
]
first = True
for t, d in steps:
    para(tf, [(t, 15, True, BLUE)], first=first, sa=2, sb=0 if first else 7)
    first = False
    para(tf, [(d, 14, False, INK)], sa=0, ls=1.14)
rect(s, MARGIN, 6.02, 5.85, 0.92, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.10)
tf = tb(s, MARGIN + 0.14, 6.08, 5.6, 0.80, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("偏置因子 ReH = (ωτH)²/(1+(ωτH)²)：默认参数仅 0.059（衰减 17 倍），收敛测试配置 0.908", 14, False, INK)],
     first=True, sa=0, ls=1.12)
pic(s, "esc_math_demod.png", 6.65, 2.42, 6.15, 4.52)

# ============================================================
# S8 双参数正交载波
# ============================================================
s = content_slide("双参数正交载波：Kp 用 sin、Kd 用 cos", "04 · 原理")
tf = tb(s, MARGIN, 1.22, 5.45, 5.7)
heading(tf, "正交载波注入（:384-385）", first=True, sb=2)
para(tf, [("Kp(t) = Kpc·(1 + a·sinωt)", 16, True, INK)], sa=2)
para(tf, [("Kd(t) = Kdc·(1 + a·cosωt)", 16, True, INK)], sa=6)
heading(tf, "正交性 ⇒ 两路互不串扰")
para(tf, [("均值(sinωt·cosωt) = ½·均值(sin2ωt) = 0（整周期严格为零）", 15, False, INK)], sa=3, ls=1.12)
para(tf, [("ĝ_Kp = 2Σ(y·sinφ)/(aN)，  ĝ_Kd = 2Σ(y·cosφ)/(aN)（:401-405）", 15, True, INK)], sa=6, ls=1.12)
heading(tf, "关键性质：测的是真梯度 ∇J")
para(tf, [("每个载波分量的系数 = 该方向全梯度分量（含 Hessian 交叉耦合 H12），不是逐坐标偏导的拼凑", 14, False, INK)], sa=6, ls=1.16)
heading(tf, "前提与边界")
para(tf, [("窗口须覆盖整数个载波周期；非整数窗的残余串扰约 0.6%，二倍频泄漏更关键（见“已知限制”）", 14, False, MUTED)], sa=0, ls=1.16)
pic(s, "esc_math_2param.png", 6.20, 1.22, 6.60, 5.70)

# ============================================================
# S9 时间尺度三分离
# ============================================================
s = content_slide("时间尺度三分离：ESC 推导的地基", "04 · 原理")
formula_box(s, MARGIN, 1.18, 12.23, 0.92, [
    [("闭环动态 τ ≈ 3/(ζωn) ≈ 18 s   ≪   载波周期 2π/ω（60 ~ 126 s）   ≪   积分器适应（数十分钟，约 18 窗）", 17, True, INK)],
])
tf = tb(s, MARGIN, 2.32, 5.70, 4.6)
bullet(tf, "闭环 ≪ 载波：每个载波周期内闭环近似到达准稳态，J(θ) 才能当作静态映射做泰勒展开", size=14, mark="• ", sa=6)
bullet(tf, "载波 ≪ 适应：θc 在一个载波周期内几乎不动，“中心值 + 摄动”的分解才有意义", size=14, sa=6)
bullet(tf, "理论背书（Krstić–Wang 2000）：平均化 + 奇异摄动 ⇒ 实用稳定，局部指数收敛到 θ* 的 O(a) 邻域", size=14, sa=6)
bullet(tf, "“局部” ⇒ ESC 只负责基线邻域微调，全局靠极点配置 —— 三层分工的理论依据", size=14, sa=6)
bullet(tf, "校核：测试配置分离比仅约 3 倍（勉强成立）；典型船 18 窗 ≈ 18 min 收敛，远慢于载波", size=14, sa=0)
pic(s, "esc_math_timescale.png", 6.45, 2.32, 6.35, 4.62)

# ============================================================
# S10 信号链与实现流程
# ============================================================
s = content_slide("信号链与实现流程", "05 · 实现")
pic(s, "esc_math_chain.png", MARGIN, 1.18, 8.25, 4.88)
tf = tb(s, MARGIN, 6.12, 8.25, 0.82)
para(tf, [("同一 escPhase_ 驱动注入与解调（因果同相）；窗口非重叠，结算后清零重计；代码位置见图底标注", 14, False, MUTED)],
     first=True, sa=0, ls=1.14)
rect(s, 9.00, 1.18, 3.78, 2.72, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 9.16, 1.30, 3.46, 2.50)
para(tf, [("每拍 50 Hz（tick :150-183）", 15, True, BLUE)], first=True, sa=4)
para(tf, [("速度门控 → 调度直通 / 周期辨识评估 → accumulate 累积 + 解调 → 窗口计数 → escDither 注入载波 → 返回增益给 PD", 14, False, INK)], sa=0, ls=1.18)
rect(s, 9.00, 4.06, 3.78, 2.88, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 9.16, 4.18, 3.46, 2.66)
para(tf, [("窗末 60 s 结算（:333-345）", 15, True, BLUE)], first=True, sa=4)
para(tf, [("ĝ 归一化 → LPF 平滑 → 积分更新 → ±10% 限幅 → 清窗 → 稳定门审查 → 接受持久化 / 回退基线", 14, False, INK)], sa=0, ls=1.18)

# ============================================================
# S11 离散化与更新律
# ============================================================
s = content_slide("离散化实现与更新律", "05 · 实现")
formula_box(s, MARGIN, 1.22, 7.15, 1.50, [
    [("每拍 HPF（后向欧拉，:369-371）", 14, True, BLUE)],
    [("y_k = αH·(y_{k-1} + c_k − c_{k-1})，  αH = τH/(τH+dt) = 5/5.02 ≈ 0.996", 15, True, INK)],
])
formula_box(s, MARGIN, 2.88, 7.15, 1.50, [
    [("窗末 LPF（:407-409）", 14, True, BLUE)],
    [("g_w = (1−αL)·g_{w-1} + αL·ĝ_w，  αL = Tw/(τL+Tw) = 60/70 ≈ 0.857", 15, True, INK)],
])
formula_box(s, MARGIN, 4.54, 7.15, 2.30, [
    [("更新律与限幅（:411-416，clampUpdate :452-461）", 14, True, BLUE)],
    [("θc ← θc − η·g，η = 0.02；  |Δθc| ≤ max(10%·|θc|, 1e-3)；负值钳 0", 15, True, INK)],
    [("收敛条件：0 < η·θc·J''·ReH < 2（逐窗线性收缩）；J'' 事先未知，η 取保守小步长", 14, False, MUTED)],
    [("远离最优点时 η·ĝ 恒超限 → 实际骑 ±10% 限幅爬坡；近段梯度变小自动进入线性区", 14, False, MUTED)],
])
rect(s, 7.95, 1.22, 4.83, 2.85, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.07)
tf = tb(s, 8.11, 1.34, 4.51, 2.62)
para(tf, [("P11 整改教训：系数必须用真实采样周期", 15, True, ACCENT)], first=True, sa=4, ls=1.12)
para(tf, [("LPF 的输入每 60 s 才来一个新样本；若误按 dt=0.02 s 设计，αL = 0.002 与正确值 0.857 差约 430 倍，"
           "梯度被衰减 2~3 个数量级，调参结果与初值逐位相同（头注释 :8-12）", 14, False, INK)], sa=0, ls=1.18)
rect(s, 7.95, 4.24, 4.83, 2.60, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 8.11, 4.36, 4.51, 2.38)
para(tf, [("±10% 限幅的双重角色", 15, True, BLUE)], first=True, sa=4)
para(tf, [("① 自适应步长归一化：等效“每窗最多 10%”的固定速率爬坡；", 14, False, INK)], sa=3, ls=1.16)
para(tf, [("② 野梯度防火墙：单窗噪声/机动畸变的错误估计最多造成 10% 偏移，且随即被稳定门审查", 14, False, INK)], sa=0, ls=1.16)

# ============================================================
# S12 安全机制四道防线
# ============================================================
s = content_slide("安全机制：四道防线", "06 · 安全")
cards = [
    ("① 稳定门 stabilityCheck（:241-254）",
     "四条件全过才接受更新：Kp > 0；ζ ≥ 0.6；ωn ≤ 0.5；振荡峰-峰 ≤ 5°。"
     "与极点配置同一组公式反解模态，配置与门限严格同口径"),
    ("② 回退基线 revert（:429-432）",
     "结算不过 → 恢复极点配置基线（非上次值），发散路径被硬切断；"
     "辨识快照无效 ⇒ 必回退：增益钉在基线，ESC 停摆但不失控"),
    ("③ 振荡检测 OscillationDetector（:257-292）",
     "误差峰-峰幅值跨完整半周估计；新值取大（保守）+ 0.9/0.1 平滑衰减；"
     "参考稳定满 30 s 才喂入（:505），机动期不误判"),
    ("④ 速度门控 + 挂起 suspend（:152, :199-203）",
     "> 15 kn 自动去激活，增益保持不再微调；切出手动时清测量状态、保留基线，"
     "防旧数据混入下一窗口"),
]
for i, (t, d) in enumerate(cards):
    x = MARGIN + (i % 2) * 6.22
    y = 1.25 + (i // 2) * 2.52
    rect(s, x, y, 6.01, 2.36, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
    tf = tb(s, x + 0.17, y + 0.12, 5.67, 2.12)
    para(tf, [(t, 15, True, BLUE)], first=True, sa=5, ls=1.1)
    para(tf, [(d, 14, False, INK)], sa=0, ls=1.17)
rect(s, MARGIN, 6.34, 12.23, 0.62, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.12)
tf = tb(s, MARGIN + 0.18, 6.38, 11.9, 0.54, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("设计原则：平均化理论不覆盖的环节（限幅 / 回退 / 门控）由确定性代数判据独立兜底 —— 安全不依赖 ESC 收敛性假设", 14, True, INK)],
     first=True, sa=0, ls=1.1)

# ============================================================
# S13 调参生命周期
# ============================================================
s = content_slide("调参生命周期：冷启动 → 收敛 → 漂移重基线", "06 · 安全")
stages = [
    ("① 冷启动", "调度表增益直通（:155）；基线未建立不注载波、不微调"),
    ("② Z 形辨识试验", "RLS 每 1 s 一拍、λ=0.98；激励门控 |δ|>3°、|r|>0.5°/s"),
    ("③ 建立基线", "辨识收官 → establishBaseline（:190）：极点配置 + 稳定门 + ESC 重锚"),
    ("④ ESC 窗口寻优", "60 s/窗梯度更新，±10% 限幅，稳定门逐窗把关，约 18 窗收敛"),
    ("⑤ 漂移重基线", "|ΔK| / |ΔT| 超 25% → 重新极点配置（:308-329，P12）"),
]
for i, (t, d) in enumerate(stages):
    x = MARGIN + i * 2.52
    rect(s, x, 1.30, 2.28, 2.42, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.08)
    tf = tb(s, x + 0.10, 1.42, 2.08, 2.20)
    para(tf, [(t, 15, True, BLUE)], first=True, sa=4, ls=1.08)
    para(tf, [(d, 14, False, INK)], sa=0, ls=1.15)
    if i < 4:
        tf = tb(s, x + 2.28, 2.28, 0.26, 0.5, anchor=MSO_ANCHOR.MIDDLE, wrap=False)
        para(tf, [("→", 18, True, ACCENT)], first=True, align=PP_ALIGN.CENTER, sa=0)
rect(s, MARGIN, 4.02, 12.23, 1.30, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.08)
tf = tb(s, MARGIN + 0.18, 4.12, 11.9, 1.10, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("门控细节：", 14, True, BLUE),
          ("漂移 25% 阈值内 ESC 自治、不动基线；显式重基线才 rebaseEsc —— 清积分器 / 解调 / HPF / 窗口，"
           "但刻意保留载波相位连续（:436-437），避免重锚拍载波突跳", 14, False, INK)], first=True, sa=3, ls=1.15)
para(tf, [("循环：", 14, True, BLUE),
          ("阶段⑤完成后回到④继续寻优；切出 MANUAL/HOLD 时 suspend，保留基线、清测量", 14, False, INK)], sa=0, ls=1.15)
rect(s, MARGIN, 5.50, 12.23, 1.42, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.08)
tf = tb(s, MARGIN + 0.18, 5.60, 11.9, 1.22, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("端到端测试佐证：", 14, True, ACCENT),
          ("IdentTrialIdentifiesAndAppliesGains —— 自动 Z 形试验收官 → 辨识 K≈0.2±0.05 / T≈8±2 → "
           "PD 增益 ≈ 极点配置 (1.6, 8.6) → 模式自动回到 AUTO_HEADING 闭环（test_autopilot_core.cpp:401-428）；"
           "两套测试 17/17 + 17/17 全部通过", 14, False, INK)], first=True, sa=0, ls=1.16)

# ============================================================
# S14 仿真验证 ①：辨识与载波
# ============================================================
s = content_slide("仿真验证 ①：RLS 辨识与载波注入", "07 · 验证")
pic(s, "esc_sim_rls.png", MARGIN, 1.22, 6.30, 4.00)
pic(s, "esc_sim_dither.png", 7.05, 1.22, 5.73, 4.00)
rect(s, MARGIN, 5.42, 12.23, 1.52, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.08)
tf = tb(s, MARGIN + 0.18, 5.52, 11.9, 1.34, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("• ", 14, True, BLUE_L), ("RLS 末值 Khat = 0.2375（偏差 +18.8%）、That = 9.5191（+19.0%）：量级正确，足以支撑极点配置基线；残差正是 ESC 的微调对象", 14, False, INK)], first=True, sa=4, ls=1.14)
para(tf, [("• ", 14, True, BLUE_L), ("±5% sin/cos 载波真实到达 PD 增益（中心 ×(1±5%) 包络内）；全程舵角受 ±35° / 10°/s 限幅保护", 14, False, INK)], sa=4, ls=1.14)
para(tf, [("• ", 14, True, BLUE_L), ("基线建立（绿虚线）后 ESC 才启动注载波 —— 载波不持久化，只作用于当拍增益输出", 14, False, INK)], sa=0, ls=1.14)

# ============================================================
# S15 仿真验证 ②：增益轨迹与代价
# ============================================================
s = content_slide("仿真验证 ②：增益轨迹与窗口代价", "07 · 验证")
pic(s, "esc_sim_gains.png", MARGIN, 1.22, 7.30, 5.30)
stats = [
    ("18 窗 converged = 1", "典型船 K=0.2 / T=8，ar_sim --tune 全链路仿真"),
    ("J：93639.7 → 93221.2", "窗口 ITAE 代价 −0.45%（y 轴截断显示窗间差异）"),
    ("Kp 2.50 → ≈1.5，Kd 12.0 → ≈7.2", "增益中心逐窗迁移，单窗更新 ≤ ±10%"),
    ("前 11 窗下探、后 7 窗平台", "进入 O(a) 邻域：限幅爬坡 → 线性收缩两段特征清晰"),
]
for i, (t, d) in enumerate(stats):
    y = 1.30 + i * 1.42
    rect(s, 8.10, y, 4.68, 1.26, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.08)
    tf = tb(s, 8.26, y + 0.10, 4.36, 1.06, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t, 15, True, BLUE)], first=True, sa=3, ls=1.08)
    para(tf, [(d, 14, False, INK)], sa=0, ls=1.12)

# ============================================================
# S16 仿真验证 ③：全链路概览
# ============================================================
s = content_slide("仿真验证 ③：自动调参全链路概览", "07 · 验证")
pic(s, "esc_sim_overview.png", MARGIN, 1.18, 5.80, 5.76)
tf = tb(s, 6.65, 1.35, 6.13, 5.5)
bullet(tf, "四联概览：航向误差 / 舵角 / 增益中心 / 窗口 J 结算点，全程 1200 s（20 min）", size=15, sa=8)
bullet(tf, "t ≈ 20 s 基线建立（绿虚线），ESC 启动注载波；增益由极点配置起点逐窗迁移", size=15, sa=8)
bullet(tf, "调参过程不打断正常保向任务：全程舵角受限（±35°）、误差有界、无发散路径", size=15, sa=8)
bullet(tf, "J 结算点阶梯式下移后进入平台 —— 收敛行为与“实用稳定到 O(a) 邻域”的理论预期一致", size=15, sa=8)
bullet(tf, "误差通道的持续摆动来自 Z 形激励与波频尺度运动：属于 J 的不可约成分，非调参失效", size=15, sa=0)

# ============================================================
# S17 已知限制
# ============================================================
s = content_slide("已知限制（诚实呈现）", "08 · 展望")
limits = [
    ("默认载波频率与窗口的结构性张力",
     "ω=0.05 rad/s（周期 125.7 s）vs Tw=60 s → 窗内仅 0.477 个载波周期，整周期正交/归一化全部失效；"
     "ReH=0.059 梯度衰减 17 倍；|Δθc| 小于约 4%·θc 后谐波泄漏淹没信号，表现为最优点附近极限环。"
     "全部收敛证据均来自成套参数 ω=2π/60（每窗恰 1 个整周期）"),
    ("HPF 衰减与相位",
     "ReH = (ωτH)²/(1+(ωτH)²) 只缩步长不改方向；任何相位失配按 cos(失配) 衰减梯度 —— "
     "代码以同一 escPhase_ 因果解调把失配压缩到只剩 HPF 相移"),
    ("J 改善 −0.45% 的正确解读",
     "保向工况 J 由不可约成分主导（波频误差、死区/量化残差、摄动自身代价 ¼J''θc²a²），"
     "微调可压缩的只是增益相关部分；方向正确 + 平台收敛即符合预期，降幅小 ≠ 失效"),
    ("Kd = 0 边界无自有激励",
     "相对摄动在 Kdc=0 处恒零，该方向梯度数学上不可观测；解锁靠 Hessian 交叉项 + 噪声相关 + absMinUpdate，"
     "方向正确性无理论保证（回归测试只锁定行为包络）"),
    ("噪声与窗口污染",
     "白噪声近似低估有色波浪噪声方差（缺波浪滤波是高海况最大短板）；机动窗 J 累积与解调无门控，"
     "当前靠稳定门 + 回退兜底"),
]
y = 1.22
for t, d in limits:
    rect(s, MARGIN, y, 12.23, 1.08, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.10)
    tf = tb(s, MARGIN + 0.16, y + 0.07, 11.55, 0.94, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t + "：", 14, True, ACCENT), (d, 14, False, INK)], first=True, sa=0, ls=1.13)
    y += 1.16

# ============================================================
# S18 改进方向
# ============================================================
s = content_slide("改进方向（按优先级）", "08 · 展望")
imps = [
    ("① 参数成套整定（最高优先级）",
     "正确顺序：先定 Tw → 取 ω = 2πn/Tw（每窗整数个载波周期）→ 配 τH 使 ReH ≥ 0.9；"
     "或把默认值改为自洽组合，并在 Params 注释显式标注“默认值需成套重配”"),
    ("② 高海况适应性",
     "增加波浪滤波，载波频率避开遭遇谱峰（0.05~0.2 Hz）；按有色谱重估梯度方差；"
     "高海况引入 dead-zone / 泄漏项抑制增益抖动传导到舵令"),
    ("③ 收敛速率归一化",
     "J 绝对量级随工况漂移，固定 η=0.02 无法全程归一化（中段速率随工况漂移）→ 自适应步长 / 梯度归一化"),
    ("④ 代价函数升级（P1 建议）",
     "操舵附加阻力型 J = E[ ψe² + λδ² ]（λ≈1/12），把舵诱导阻力口径直接写入代价"),
    ("⑤ 边界与污染修补",
     "Kd=0 通道改加性摄动 Kd = Kdc + a·sinωt（数学上恢复该方向激励）；参考机动后冻结本窗解调累积，杜绝机动窗 J 污染"),
]
y = 1.22
for t, d in imps:
    rect(s, MARGIN, y, 12.23, 1.08, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.10)
    tf = tb(s, MARGIN + 0.16, y + 0.07, 11.55, 0.94, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t + "：", 14, True, BLUE), (d, 14, False, INK)], first=True, sa=0, ls=1.13)
    y += 1.16

# ============================================================
# S19 总结
# ============================================================
s = content_slide("总结", "09 · 总结")
sums = [
    ("架构", "三层调参分工：调度表粗调 → 极点配置给基线 → ESC 只做基线邻域内的局部微调；"
             "“局部”是平均化理论的边界，也是架构分工的依据"),
    ("原理", "摄动 → 解调 → 积分：ĝ = 2Σ(y·sinφ)/(aN)；sin/cos 正交载波实现 Kp/Kd 双参数无串扰梯度估计；"
             "时间尺度三分离是全部推导成立的地基"),
    ("工程", "四道防线（稳定门 / 回退 / 振荡检测 / 速度门控）+ 全生命周期门控；"
             "仿真 18 窗收敛、J −0.45%、全程舵角受限；测试 17/17 + 17/17 通过"),
]
for i, (t, d) in enumerate(sums):
    x = MARGIN + i * 4.15
    rect(s, x, 1.30, 3.94, 3.05, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
    rect(s, x, 1.30, 3.94, 0.56, fill=BLUE, rounded=True, radius=0.07)
    tf = tb(s, x, 1.32, 3.94, 0.52, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t, 17, True, WHITE)], first=True, align=PP_ALIGN.CENTER, sa=0)
    tf = tb(s, x + 0.18, 2.02, 3.58, 2.24)
    para(tf, [(d, 14, False, INK)], first=True, sa=0, ls=1.2)
rect(s, MARGIN, 4.62, 12.23, 0.98, fill=BLUE, rounded=True, radius=0.10)
tf = tb(s, MARGIN + 0.2, 4.70, 11.85, 0.82, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("ESC 的价值不在“调得更多”，而在 在线、免模型、带安全兜底 地持续微调 —— 把节能口径直接变成调参目标", 16, True, WHITE)],
     first=True, align=PP_ALIGN.CENTER, sa=0, ls=1.15)
tf = tb(s, MARGIN, 5.86, 12.23, 1.0)
para(tf, [("谢谢，欢迎讨论", 18, True, BLUE)], first=True, align=PP_ALIGN.CENTER, sa=6)
para(tf, [("参考：.scratch/esc_report/math_derivation.md · code_facts.md · control/include/auto_tuner.hpp · docs/工程总体分析与ESC方案原理.md", 14, False, MUTED)],
     align=PP_ALIGN.CENTER, sa=0)

os.makedirs(OUT_DIR, exist_ok=True)
prs.save(OUT)
print(f"saved: {OUT}, slides={len(prs.slides._sldIdLst)}")
