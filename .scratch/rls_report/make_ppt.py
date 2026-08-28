#!/usr/bin/env python3
"""make_ppt.py — 生成《RLS 在线辨识方案汇报》PPT（16:9，17 页）

素材来源：.scratch/rls_report/math_derivation.md、code_facts.md、assets/rls_*.png
输出：ppt_output/RLS方案汇报.pptx
用法：.venv-ppt/bin/python .scratch/rls_report/make_ppt.py
注意：避免组合变音符（K̂），一律用 Khat/That 写法。
"""
import os
import copy
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
OUT = os.path.join(OUT_DIR, "RLS方案汇报.pptx")

# ---------- 设计规范：与《ESC方案汇报.pptx》一致 ----------
BLUE = "1F4E79"
BLUE_D = "17375E"
BLUE_L = "2E75B6"
ACCENT = "D9822B"
INK = "262626"
MUTED = "595959"
FAINT = "7F7F7F"
BOX_BG = "EEF3FA"
BOX_LINE = "B8CCE4"
WARN_BG = "FDF3E7"
WARN_LINE = "E0C090"
WHITE = "FFFFFF"
TITLE_SUB = "BBD2EA"

FONT = "Microsoft YaHei"

# 独立公式统一格式：非加粗、深蓝、14pt（原各行 14/15/16/17 不一，取多数派 14）
FORMULA_SIZE = 14
FORMULA_COLOR = BLUE

EMU_W, EMU_H = 12192000, 6858000
PAGE_W, PAGE_H = 13.3333, 7.5
MARGIN = 0.55

prs = Presentation()
prs.slide_width = Emu(EMU_W)
prs.slide_height = Emu(EMU_H)
BLANK = prs.slide_layouts[6]

TOTAL_PAGES = 17
_page_no = 0


def _set_font(run, name):
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
    rect(s, MARGIN, 7.09, PAGE_W - 2 * MARGIN, 0.012, fill="D9D9D9")
    tf = tb(s, MARGIN, 7.13, 9.5, 0.32)
    para(tf, [("RLS 在线辨识方案汇报 · 船舶自动舵", 14, False, FAINT)], first=True, sa=0)
    tf = tb(s, 11.4, 7.13, 1.38, 0.32)
    para(tf, [(f"{_page_no} / {TOTAL_PAGES}", 14, False, FAINT)], first=True,
         align=PP_ALIGN.RIGHT, sa=0)
    return s


def heading(tf, text, first=False, sa=4, sb=6, size=17, color=BLUE):
    return para(tf, [(text, size, True, color)], first=first, sa=sa, sb=sb)


def bullet(tf, text, size=14, sa=4, color=INK, bold=False, mark="• "):
    return para(tf, [(mark, size, True, BLUE_L), (text, size, bold, color)], sa=sa, ls=1.1)


def formula_box(slide, x, y, w, h, lines, bg=BOX_BG, line=BOX_LINE):
    rect(slide, x, y, w, h, fill=bg, line=line, rounded=True, radius=0.06)
    tf = tb(slide, x + 0.12, y + 0.06, w - 0.24, h - 0.12, anchor=MSO_ANCHOR.MIDDLE)
    for i, runs in enumerate(lines):
        # 在 run 元组层统一公式格式（保留 italic 等尾部字段）；
        # 后处理的上下标拆分深拷贝 rPr，自动继承同一字号/颜色/非加粗。
        runs = [(item[0], FORMULA_SIZE, False, FORMULA_COLOR) + tuple(item[4:])
                for item in runs]
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
para(tf, [("RLS 在线辨识方案汇报", 40, True, WHITE)], first=True, sa=0)
rect(s, 1.02, 3.42, 2.2, 0.06, fill=ACCENT)
tf = tb(s, 1.0, 3.62, 11.3, 0.6)
para(tf, [("带遗忘因子 λ=0.98 的递归最小二乘 · 在线辨识 Nomoto 参数 K/T", 20, False, "D7E3F2")],
     first=True, sa=0)
tf = tb(s, 1.0, 4.55, 11.3, 1.5)
para(tf, [("汇报依据：control/include/nomoto_identifier.hpp 实现 · 数学推导与代码事实（.scratch/rls_report/）", 14, False, "A9C3E0")], first=True, sa=5)
para(tf, [("仿真验证：ar_sim --ident / --trial（典型船 K=0.2 / T=8，Z 形 ±40°/15 s 激励）", 14, False, "A9C3E0")], sa=5)
para(tf, [("2026-08-21", 14, False, "A9C3E0")], sa=0)
tf = tb(s, 1.0, 6.86, 11.3, 0.4)
para(tf, [("Recursive Least Squares · λ = 0.98 · 三层调参架构 L2 层", 14, False, "7FA3CC")],
     first=True, align=PP_ALIGN.CENTER, sa=0)

# ============================================================
# S2 目录
# ============================================================
s = content_slide("目录", "CONTENTS")
toc = [
    ("01", "背景与问题：为什么需要在线辨识 · 三层调参架构中的 L2"),
    ("02", "被控对象建模：一阶 Nomoto 模型与 K / T 物理意义"),
    ("03", "回归化与归一化：y = φᵀθ · 条件数治理"),
    ("04", "加权最小二乘目标与正规方程"),
    ("05", "RLS 递推三式：增益 / 新息 / 更新 / 协方差"),
    ("06", "遗忘因子 λ=0.98：记忆窗口 · P 发散与保底"),
    ("07", "激励条件与门控：PE · 稳态直航 T 欠定"),
    ("08", "自降频与工程防御：1 Hz · 差分降噪 · 锚点保护"),
    ("09", "实现流程与下游消费：极点配置 · 稳定门 · 漂移门"),
    ("10", "仿真验证 · 已知限制 · 改进方向 · 总结"),
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
s = content_slide("背景与问题：为什么需要在线辨识", "01 · 背景")
tf = tb(s, MARGIN, 1.25, 5.75, 3.95)
heading(tf, "K / T 不是常数", first=True)
bullet(tf, "水动力导数随船型、航速、载况、水深、海况变化 —— K/T 慢时变，一次离线标定无法覆盖全工况")
bullet(tf, "极点配置公式 Kp = ωn²·T/K、Kd = (2ζωnT − 1)/K 直接以 Khat/That 为输入：辨识误差按比例传入增益")
bullet(tf, "Khat 偏小 25% ⇒ Kp、Kd 偏大 33% —— 辨识精度是三层架构基线质量的定量来源")
para(tf, [("方案定位：", 14, True, ACCENT),
          ("用带遗忘因子 λ=0.98 的 RLS，从舵角 δ 与艏摇率 r 在线辨识 K/T，供下游建基线与稳定门校验", 14, False, INK)],
     sb=6, sa=0, ls=1.12)

layers = [
    ("L1  增益调度表（粗调）", "速度 × 海况查表名义值 · gain_schedule.hpp（不依赖 K/T）", BOX_BG, BLUE),
    ("L2  RLS 辨识 + 极点配置（本方案）", "Khat/That → Kp = ωn²That/Khat 建基线 · nomoto_identifier.hpp / auto_tuner.hpp:228-237", WARN_BG, ACCENT),
    ("L3  ESC 在线微调", "基线邻域内梯度寻优，吸收模型残差 · auto_tuner.hpp", BOX_BG, BLUE),
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
para(tf, [("辨识结果的消费方式：", 15, True, BLUE),
          ("辨识器只输出只读快照 IdSnapshot{excited, valid, K, T}（auto_tuner.hpp:31-36），"
           "不直接改增益 —— 先过 valid 物理门，再经极点配置 + 稳定门 + ±25% 漂移门三重闸才生效（autopilot_core.hpp:176-178）",
           15, False, INK)], first=True, sa=0, ls=1.15)

# ============================================================
# S4 被控对象建模
# ============================================================
s = content_slide("被控对象建模：一阶 Nomoto 模型", "02 · 模型")
tf = tb(s, MARGIN, 1.22, 6.35, 5.7)
heading(tf, "从操纵性方程到 Nomoto", first=True, sb=2)
para(tf, [("Abkowitz 线性模型（横荡 v + 艏摇 r 两自由度）对横荡取准定常近似、消去 v，得集总形式：", 14, False, MUTED)], sa=3, ls=1.14)
para(tf, [("I·ṙ + N·r = M·δ   ⇒   T·ṙ + r = K·δ，T = I/N，K = M/N", FORMULA_SIZE, False, FORMULA_COLOR)], sa=4, ls=1.1)
para(tf, [("I 有效艏摇惯性、N 等效艏摇阻尼、M 舵效力矩系数；航向稳定船 N>0 ⇒ K>0、T>0（valid() 门限的力学依据，:119-124）", 14, False, MUTED)], sa=6, ls=1.14)
heading(tf, "传递函数与阶跃响应")
para(tf, [("r(s)/δ(s) = K/(T·s+1)；ψ(s)/δ(s) = K/[s·(T·s+1)]", FORMULA_SIZE, False, FORMULA_COLOR)], sa=3, ls=1.1)
para(tf, [("r(t) = K·δ₀·(1 − e^(−t/T))：稳态增益管 K、上升沿管 T —— 两个参数各管一件事", 14, False, MUTED)], sa=6, ls=1.14)
para(tf, [("这决定了可辨识性结构：K 由稳态段数据决定，T 只能由 ṙ≠0 的瞬态段数据决定（第 9 页门控设计的根源）", 14, True, ACCENT)], sa=0, ls=1.14)

rect(s, 7.15, 1.22, 5.63, 2.42, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 7.33, 1.34, 5.27, 2.20)
para(tf, [("K — 旋回性指数（1/s）", 15, True, BLUE)], first=True, sa=2)
para(tf, [("单位舵角产生的稳态转向率；取值 (0,2]，典型 0.05~0.5", 14, False, INK)], sa=6, ls=1.14)
para(tf, [("T — 追随性指数（s）", 15, True, BLUE)], sa=2)
para(tf, [("艏摇惯性/阻尼时间尺度，响应滞后舵角约 T 秒；取值 [1,60]，覆盖快艇到大型满载油轮", 14, False, INK)], sa=0, ls=1.14)
rect(s, 7.15, 3.84, 5.63, 1.55, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 7.33, 3.96, 5.27, 1.33)
para(tf, [("为什么能在线辨识", 15, True, BLUE)], first=True, sa=3)
para(tf, [("K、T 以线性方式出现在回归式中 ⇒ 整个问题归结为（加权）线性最小二乘，无需非线性迭代 —— 这是选 RLS 的数学原因", 14, False, INK)], sa=0, ls=1.16)
rect(s, 7.15, 5.56, 5.63, 1.36, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.08)
tf = tb(s, 7.33, 5.66, 5.27, 1.16, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("模型边界：", 14, True, ACCENT),
          ("一阶 Nomoto 是多重近似的产物，横荡动态、舵机饱和/死区、风浪外力矩均在模型外 —— 失配由下游稳定门兜底", 14, False, INK)],
     first=True, sa=0, ls=1.14)

# ============================================================
# S5 回归化与归一化
# ============================================================
s = content_slide("回归化与归一化：从微分方程到 y = φᵀθ", "03 · 回归")
formula_box(s, MARGIN, 1.18, 12.23, 1.30, [
    [("后向差分 dr_k = (r_k − r_{k−1})/Δt 代入 T·ṙ + r = K·δ，整理得线性参数回归（:3-4, :65-67, :76, :88）", 14, True, BLUE)],
    [("y_k = φ_kᵀ·θ：  y_k = r_k，  φ_k = [δ_k/50,  −dr_k]ᵀ，  θ = [K, T]ᵀ", 17, True, INK)],
    [("归一化域：θ₀ = 50K、θ₁ = T；估计完成后反归一化 K = θ₀/50、T = θ₁（:112-115）", 14, False, MUTED)],
])
tf = tb(s, MARGIN, 2.70, 5.55, 4.2)
heading(tf, "为什么要归一化（dScale=50 / drScale=1）", first=True, sb=2, size=16)
bullet(tf, "原始回归分量量级悬殊：δ 幅度 3°~35°，dr 幅度 0.2~1 deg/s²，信息矩阵对角元之比达 (10/0.5)² ≈ 400，工况变化可达 10³~10⁴", size=14, sa=6)
bullet(tf, "归一化后 φ₀∈[−0.7,0.7]、φ₁ 为 O(1)，条件数压到 O(1~10²)，协方差初值可取统一 p0 = 1（:23-26）", size=14, sa=6)
bullet(tf, "归一化不改变最小二乘的数学解，只改变数值条件与 P 阵坐标标度 —— P 保底 1e-3 也因此在归一化坐标下才有统一意义", size=14, sa=6)
para(tf, [("代价：", 14, True, ACCENT),
          ("差分放大噪声 σ_ν = √2·σ_r/Δt —— 50 Hz 直接差分噪声放大 70 倍，靠第 10 页的 1 Hz 自降频压回可用水平", 14, False, INK)], sa=0, ls=1.14)
pic(s, "rls_math_regression.png", 6.25, 2.70, 6.55, 4.20)

# ============================================================
# S6 加权最小二乘目标与正规方程
# ============================================================
s = content_slide("加权最小二乘目标与正规方程", "04 · 最小二乘")
formula_box(s, MARGIN, 1.22, 7.05, 1.72, [
    [("指数加权目标（λ = 0.98，:18）", 14, True, BLUE)],
    [("J_N(θ) = Σ_{k=1..N} λ^(N−k)·(y_k − φ_kᵀθ)²", 17, True, INK)],
    [("当前样本权重 1，a 拍前样本权重 λᵃ；λ=1 退化为普通最小二乘", 14, False, MUTED)],
])
formula_box(s, MARGIN, 3.10, 7.05, 1.90, [
    [("梯度置零 ⇒ 加权正规方程", 14, True, BLUE)],
    [("R_N·θ = r_N：  R_N = Σ λ^(N−k)·φ_kφ_kᵀ，  r_N = Σ λ^(N−k)·φ_k·y_k", 16, True, INK)],
    [("Hessian = 2R_N：R_N 正定 ⇒ 严格凸、解唯一且全局最小：θ_N = R_N⁻¹·r_N", 14, False, MUTED)],
])
formula_box(s, MARGIN, 5.16, 7.05, 1.76, [
    [("递推结构（递推域的遗忘读法）", 14, True, BLUE)],
    [("R_N = λ·R_{N−1} + φ_Nφ_Nᵀ，  r_N = λ·r_{N−1} + φ_N·y_N", 16, True, INK)],
    [("旧信息整体乘 λ 衰减一拍，再叠加新信息 —— 秩一更新引出下一页的矩阵求逆引理", 14, False, MUTED)],
])
rect(s, 7.95, 1.22, 4.83, 2.72, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 8.11, 1.34, 4.51, 2.50)
para(tf, [("为什么要遗忘因子", 15, True, BLUE)], first=True, sa=4)
para(tf, [("等权 LS 隐含“参数恒定”假设：P_N ~ O(1/N)→0、增益 L_N→0，估计器“睡死”，无法跟踪工况漂移", 14, False, INK)], sa=4, ls=1.18)
para(tf, [("K/T 随航速、载况、水深慢变 —— 需要“旧数据逐渐遗忘、新数据主导”的估计器", 14, False, INK)], sa=0, ls=1.18)
rect(s, 7.95, 4.10, 4.83, 2.82, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 8.11, 4.22, 4.51, 2.60)
para(tf, [("为什么选递推而非批处理", 15, True, BLUE)], first=True, sa=4)
para(tf, [("批处理每拍要解 2×2 线性方程组并存全部历史；递推形式每拍 O(n²) 更新、无需存历史，且天然携带协方差信息", 14, False, INK)], sa=4, ls=1.18)
para(tf, [("2 维情形每拍仅约 20 次 FLOP，1 Hz 节拍对 ECU 完全无负担", 14, False, MUTED)], sa=0, ls=1.18)

# ============================================================
# S7 RLS 递推三式
# ============================================================
s = content_slide("RLS 递推三式：推导要点与代码对照", "04 · 递推")
formula_box(s, MARGIN, 1.18, 12.23, 2.30, [
    [("对 R_N = λR_{N−1} + φφᵀ 用矩阵求逆引理（Woodbury），记 P_N = R_N⁻¹，逐步推出四式（2 维显式展开，无矩阵库依赖）", 14, True, BLUE)],
    [("增益   L_N = P_{N−1}φ_N / (λ + φ_NᵀP_{N−1}φ_N)                    （:79-85，denom 除零保护 1e-12）", 15, True, INK)],
    [("新息   e_N = y_N − φ_Nᵀθ_{N−1}          参数更新   θ_N = θ_{N−1} + L_N·e_N          （:88-94）", 15, True, INK)],
    [("协方差   P_N = (1/λ)·(P_{N−1} − L_N·φ_NᵀP_{N−1})          （:97-103，对角保底 1e-3）", 15, True, INK)],
    [("关键恒等式 P_Nφ_N = L_N：增益方向 = 新回归量经最新协方差加权后的方向；P 对角元 ∝ 参数估计方差", 14, False, MUTED)],
])
pic(s, "rls_math_update.png", MARGIN, 3.62, 12.23, 3.32)

# ============================================================
# S8 遗忘因子与 P 保底
# ============================================================
s = content_slide("遗忘因子 λ=0.98：记忆窗口与 P 的两个极端", "05 · 遗忘")
tf = tb(s, MARGIN, 1.22, 5.55, 3.4)
heading(tf, "有效记忆窗口", first=True, sb=2, size=16)
para(tf, [("N_eff = 1/(1−λ) = 50 样本 ≈ 50 s（RLS 拍 1 s）—— 恰好覆盖 Z 形机动 2~3 个换舵周期", 14, False, INK)], sa=3, ls=1.14)
para(tf, [("半衰期 ln2/ln(1/λ) ≈ 34.3 拍；加权样本平均年龄 λ/(1−λ) = 49 拍（跟踪滞后来源）", 14, False, MUTED)], sa=6, ls=1.14)
heading(tf, "偏差–方差权衡", size=16)
bullet(tf, "λ 小：窗口短、跟踪快，但有效样本少、估计方差大", size=14)
bullet(tf, "λ=0.98：稳态方差压缩系数 (1−λ)/(1+λ) ≈ 0.0101（强滤波），代价是约 49 拍滞后 —— 对分钟级船况漂移够用，秒级突变由重基线兜底", size=14, sa=0)
rect(s, MARGIN, 4.72, 5.55, 2.20, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.07)
tf = tb(s, MARGIN + 0.16, 4.84, 5.23, 1.98)
para(tf, [("P 的两个极端与防护", 15, True, ACCENT)], first=True, sa=4)
para(tf, [("发散：无激励时若仍更新，P ≈ P/λ 每拍放大 2.04%，500 拍膨胀 2.2 万倍 ⇒ 门控整拍跳过、P 冻结", 14, False, INK)], sa=4, ls=1.16)
para(tf, [("塌陷：持续激励下 P→0、增益熄零 ⇒ 对角保底 max(P_ii, 1e-3)（:98,105-106），估计器“永不睡死”", 14, False, INK)], sa=0, ls=1.16)
pic(s, "rls_math_forgetting.png", 6.25, 1.22, 6.55, 5.70)

# ============================================================
# S9 激励条件与门控
# ============================================================
s = content_slide("激励条件与门控：持续激励（PE）的在线实现", "06 · 激励")
formula_box(s, MARGIN, 1.18, 12.23, 1.06, [
    [("门控判据（:68-69）：本拍更新  ⇔  |δ| > 3.0°  且  (|r| > 0.5°/s  或  |dr| > 0.2°/s²)", 17, True, INK)],
    [("未过门整拍跳过、θ/P 冻结（同时防止 P 发散）；PE 条件要求回归序列不能躺在过原点的一条直线附近", 14, False, MUTED)],
])
tf = tb(s, MARGIN, 2.44, 5.55, 4.5)
heading(tf, "为什么稳态直航时 T 欠定", first=True, sb=2, size=16)
para(tf, [("直航 ṙ≈0 ⇒ dr≈0 ⇒ φ ≈ [δ/50, 0]ᵀ 全躺在坐标轴上，R_N 最小特征值恒为 0，正规方程退化为 r = Kδ：只能定 K，T 方向完全无信息", 14, False, INK)], sa=6, ls=1.16)
heading(tf, "三条门控各管一件事", size=16)
bullet(tf, "|δ|>3°：φ₀ 分量能量远离噪声底，保证“输入在激励系统”", size=14, sa=5)
bullet(tf, "|r|>0.5°/s：输出信号超出噪声，保证新息信噪比（K 方向信息质量）", size=14, sa=5)
bullet(tf, "|dr|>0.2°/s²（或条件）：专门保留 r 过零附近的高 dr 瞬态 —— T 信息密度最高的样本恰是 |r| 最小的样本；没有这一条 T 依旧欠定", size=14, sa=0)
pic(s, "rls_math_pe.png", 6.25, 2.44, 6.55, 4.48)

# ============================================================
# S10 自降频与工程防御
# ============================================================
s = content_slide("自降频与工程防御：让差分在物理上可用", "07 · 工程")
cards = [
    ("① 自降频 50 Hz → 1 Hz（:57-61）",
     "内部累计真实间隔、满 1 s 才递推一拍（点采样）。辨识对象带宽 1/(2πT) = 0.0027~0.159 Hz，"
     "对 0.5 Hz Nyquist 裕度 ≥3 倍（典型船 30 倍）：1 Hz 节拍对一阶 Nomoto 辨识带宽充分"),
    ("② 差分降噪 50 倍（核心收益）",
     "dr 噪声标准差 ∝ 1/Δt：Δt 从 20 ms 放大到 1 s，噪声压低 50 倍、SNR 提升 2500 倍。"
     "σ_r=0.1°/s 时 50 Hz 差分噪声 7.1°/s²（比门限大 35 倍、完全淹没信号），1 Hz 仅 0.14°/s²"),
    ("③ Δt 自测（:58-61）",
     "差分用两次 RLS 拍之间的真实间隔而非名义常数：调用频率偏离 50 Hz 时若用名义 Δt，"
     "T 会静默地错一个比例因子且无任何报错 —— 自测把这个语义陷阱从设计上删除"),
    ("④ 入口防御与差分锚点管理（:50-55, :70-72）",
     "NaN/Inf、|δ|>35°、|r|>30°/s、dt≤0 直接丢弃且不写 prevR_ —— 毛刺若成差分锚点，"
     "会被放大成虚假大 dr、恰好骗过 |dr| 门控永久毒化 R_N；未激励拍则正常写锚点维持点采样语义"),
]
for i, (t, d) in enumerate(cards):
    x = MARGIN + (i % 2) * 6.22
    y = 1.25 + (i // 2) * 2.62
    rect(s, x, y, 6.01, 2.46, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
    tf = tb(s, x + 0.17, y + 0.12, 5.67, 2.22)
    para(tf, [(t, 15, True, BLUE)], first=True, sa=5, ls=1.1)
    para(tf, [(d, 14, False, INK)], sa=0, ls=1.17)
rect(s, MARGIN, 6.52, 12.23, 0.44, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.16)
tf = tb(s, MARGIN + 0.18, 6.55, 11.9, 0.38, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("降频不是为了省算力（每拍仅约 20 次 FLOP），而是为了让差分这一回归分量在物理上可用", 14, True, INK)],
     first=True, sa=0, ls=1.1)

# ============================================================
# S11 实现流程与下游消费
# ============================================================
s = content_slide("实现流程与下游消费", "08 · 链路")
pic(s, "rls_math_chain.png", MARGIN, 1.18, 8.25, 4.88)
tf = tb(s, MARGIN, 6.12, 8.25, 0.82)
para(tf, [("传感器 (δ, r) → 入口校验 → 自降频 → 差分 → 门控 → 归一化回归 → RLS 三式 → valid 物理门 → 只读快照；代码行号见图底标注", 14, False, MUTED)],
     first=True, sa=0, ls=1.14)
rect(s, 9.00, 1.18, 3.78, 2.72, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 9.16, 1.30, 3.46, 2.50)
para(tf, [("每拍数据流（50 Hz 主循环）", 15, True, BLUE)], first=True, sa=4)
para(tf, [("headingControlStep 每拍无条件 update()（autopilot_core.hpp:163）→ 组装 IdSnapshot → tuner.tick() 周期评估（auto_tuner.hpp:158-161，excited && valid 才进入）", 14, False, INK)], sa=0, ls=1.18)
rect(s, 9.00, 4.06, 3.78, 2.88, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 9.16, 4.18, 3.46, 2.66)
para(tf, [("下游三重闸（auto_tuner.hpp）", 15, True, BLUE)], first=True, sa=4)
para(tf, [("极点配置 Kp = ωn²That/Khat、Kd = (2ζωnThat − 1)/Khat（:228-237，ζ=0.85、ωn=0.20）→ 稳定门 ζ≥0.6、ωn≤0.5、振荡≤5°（:241-254）→ ±25% 漂移门（:308-313）：门内 ESC 自治，越门显式重基线", 14, False, INK)], sa=0, ls=1.18)

# ============================================================
# S12 仿真验证 ①：收敛曲线
# ============================================================
s = content_slide("仿真验证 ①：Khat / That 收敛曲线", "09 · 验证")
pic(s, "rls_sim_convergence.png", MARGIN, 1.22, 7.30, 5.30)
stats = [
    ("valid 13 s 置位", "Z 形 ±40°/15 s 航向参考，t≈13 s 进入物理门限域，同拍极点配置建立基线"),
    ("--ident：Khat=0.2293（+14.7%）", "That=9.0723（+13.4%），真值 K=0.2 / T=8；valid=1、excited=1，PASS"),
    ("收敛速率与理论一致", "误差包络时间常数 ≈ 1/(1−λ) = 50 拍；约 80 s 进入真值 ±25% 容差带"),
    ("测试基座全绿", "test_nomoto_identifier 9/9 · test_autopilot_core(Ident) 2/2 · test_auto_tuner 17/17，全部 PASS"),
]
for i, (t, d) in enumerate(stats):
    y = 1.30 + i * 1.42
    rect(s, 8.10, y, 4.68, 1.26, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.08)
    tf = tb(s, 8.26, y + 0.10, 4.36, 1.06, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t, 15, True, BLUE)], first=True, sa=3, ls=1.08)
    para(tf, [(d, 14, False, INK)], sa=0, ls=1.12)

# ============================================================
# S13 仿真验证 ②：Z 形试验与门控
# ============================================================
s = content_slide("仿真验证 ②：Z 形辨识试验与激励门控", "09 · 验证")
pic(s, "rls_sim_zigzag.png", MARGIN, 1.22, 6.45, 3.60)
pic(s, "rls_sim_excitation.png", MARGIN, 4.88, 6.45, 2.06)
rect(s, 7.25, 1.22, 5.53, 2.60, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 7.43, 1.34, 5.17, 2.38)
para(tf, [("自动 Z 形试验（--trial，P14）", 15, True, BLUE)], first=True, sa=4)
para(tf, [("冷启动增益 1.0/0.5、调参器去激活；±20° 舵 / ±20° 航向换舵、4 循环", 14, False, INK)], sa=4, ls=1.16)
para(tf, [("175 s 收官：Khat = 0.2067（+3.4%）、That = 8.13（+1.6%），valid=1", 14, True, INK)], sa=4, ls=1.16)
para(tf, [("增益自动应用：Kp = 1.5723、Kd = 8.5277（极点配置 ζ=0.85、ωn=0.20），applied=1，PASS（±25%）", 14, False, INK)], sa=0, ls=1.16)
rect(s, 7.25, 3.98, 5.53, 2.94, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.07)
tf = tb(s, 7.43, 4.10, 5.17, 2.72)
para(tf, [("门控着色的信息学读法", 15, True, BLUE)], first=True, sa=4)
para(tf, [("r 过零 + 高 |dr| 瞬态 7 处（|dr|≈0.5°/s² > 0.2 门限）被“或”条件保留 —— 它们是 T 可辨识的唯一信息源", 14, False, INK)], sa=4, ls=1.16)
para(tf, [("舵角过零/初段小舵角 8 拍被拒绝：φ 能量不足，整拍跳过，P 冻结不发散", 14, False, INK)], sa=4, ls=1.16)
para(tf, [("教训（sim --esc 场景注释 :207-212）：旧 ±10°/30 s 阶跃多数样本过不了门 → 辨识有偏、基线假阳性；现用 Z 形 ±40°/15 s", 14, False, MUTED)], sa=0, ls=1.16)

# ============================================================
# S14 仿真验证 ③：三船型对比
# ============================================================
s = content_slide("仿真验证 ③：三船型收敛对比", "09 · 验证")
pic(s, "rls_sim_ships.png", MARGIN, 1.22, 7.30, 5.30)
rows = [
    ("典型船  K=0.2 / T=8（leg 15 s）", "+18.2% / +18.5%", "valid @ 13 s"),
    ("快船    K=0.5 / T=3（leg 15 s）", "+5.9% / +4.1%", "valid @ 27 s"),
    ("慢船    K=0.1 / T=20（leg 60 s）", "+2.2% / +1.6%", "valid @ 28 s"),
]
tf = tb(s, 8.10, 1.30, 4.68, 0.42)
para(tf, [("t=300 s 处 K/T 相对误差", 15, True, BLUE)], first=True, sa=0)
for i, (a, b, c) in enumerate(rows):
    y = 1.82 + i * 1.20
    rect(s, 8.10, y, 4.68, 1.06, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.08)
    tf = tb(s, 8.26, y + 0.08, 4.36, 0.90, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(a, 14, True, INK)], first=True, sa=2, ls=1.05)
    para(tf, [("误差 ", 14, False, INK), (b, 14, True, ACCENT), ("   " + c, 14, False, MUTED)], sa=0, ls=1.05)
rect(s, 8.10, 5.50, 4.68, 1.42, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.08)
tf = tb(s, 8.26, 5.60, 4.36, 1.22, anchor=MSO_ANCHOR.MIDDLE)
para(tf, [("要点：", 14, True, ACCENT),
          ("慢船 Z 形参考翻转周期须放大至 60 s（≥2T），否则只能观测初始瞬态、T 欠定；三者均远优于 ±25% 漂移门", 14, False, INK)],
     first=True, sa=0, ls=1.14)

# ============================================================
# S15 已知限制
# ============================================================
s = content_slide("已知限制（诚实呈现）", "10 · 展望")
limits = [
    ("EIV 衰减偏差",
     "差分噪声进入回归量本身构成 errors-in-variables：估计向零收缩，系数 ≈ s²/(s²+σ²)。"
     "1 Hz 拍 + Z 形激励下 T 存在约 10% 量级系统性偏低倾向；门控选择性采样与降频是对冲手段，不是根治"),
    ("1 Hz 混叠风险",
     "无前置抗混叠滤波：>0.5 Hz 的量测噪声（风浪高频、传感器噪声）混叠进辨识带宽；"
     "目前靠差分信噪比改善与 λ=0.98 的 50 样本等效平均部分对冲"),
    ("慢船 + 快换向激励下 T 欠定",
     "换向周期 ≪ 2T 时只能观测初始瞬态，φ 两个方向喂不饱；仿真要求慢船 leg ≥ 2T（60 s），"
     "在线常规操舵场景无此保证，靠门控拒更新 + 下游三重闸兜底"),
    ("稳态误差点 0.0101",
     "遗忘因子把稳态误差钉在非零点：系数 (1−λ)/(1+λ) ≈ 1.0101%，即使参数恒定、数据无限也不收缩到真值；"
     "P 保底 1e-3 又给误差点加了下界 —— 这是用渐近最优性换跟踪活性的有意交换"),
    ("P 保底的次优性",
     "保底后 P 不再等于加权信息矩阵的严格逆，算法从“有限记忆最优估计”退化为“有界增益次优估计器”；"
     "非对角元无保底、无显式对称化，长期浮点累积下对称性漂移未防护（测试未暴露问题）"),
]
y = 1.22
for t, d in limits:
    rect(s, MARGIN, y, 12.23, 1.08, fill=WARN_BG, line=WARN_LINE, rounded=True, radius=0.10)
    tf = tb(s, MARGIN + 0.16, y + 0.07, 11.55, 0.94, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t + "：", 14, True, ACCENT), (d, 14, False, INK)], first=True, sa=0, ls=1.13)
    y += 1.16

# ============================================================
# S16 改进方向
# ============================================================
s = content_slide("改进方向（按优先级）", "10 · 展望")
imps = [
    ("① 抗混叠前置低通（最高优先级）",
     "在 1 Hz 点采样前对 r 加截止 0.2~0.3 Hz 的一/二阶低通，消除高频噪声混叠；"
     "同时降低差分噪声底，直接改善 EIV 衰减系数"),
    ("② EIV 偏差修正",
     "总最小二乘（TLS）或工具变量（IV）替代普通 LS 口径；或按估计 σ_ν 对 R_N 做噪声补偿（bias-compensated RLS），"
     "把 T 的 10% 量级系统性偏低压到 1~2%"),
    ("③ 自适应遗忘因子",
     "λ 随激励/新息统计在 0.98~0.995 间调节：机动期加快跟踪、稳态期加大滤波；"
     "与门控联动，缓解“偏差–方差”固定权衡"),
    ("④ 激励编排与 T 欠定防护",
     "辨识试验按当前 That 自适应选 Z 形 leg 时长（≥2T）；在线监测 R_N 两个特征方向能量比，"
     "T 方向长期饥饿时主动提示或安排短时机动"),
    ("⑤ 数值健壮性",
     "P 更新改 UD 分解或显式对称化 P ← (P+Pᵀ)/2；非对角元监控；门控阈值与 p0/保底值随噪声水平在线标定"),
]
y = 1.22
for t, d in imps:
    rect(s, MARGIN, y, 12.23, 1.08, fill=BOX_BG, line=BOX_LINE, rounded=True, radius=0.10)
    tf = tb(s, MARGIN + 0.16, y + 0.07, 11.55, 0.94, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, [(t + "：", 14, True, BLUE), (d, 14, False, INK)], first=True, sa=0, ls=1.13)
    y += 1.16

# ============================================================
# S17 总结
# ============================================================
s = content_slide("总结", "11 · 总结")
sums = [
    ("原理", "T·ṙ + r = K·δ 后向差分即线性回归 y = φᵀθ；指数加权 λ=0.98 + 矩阵求逆引理推出 RLS 四式；"
             "归一化把条件数从 10³~10⁴ 压到 O(1~10²)，每拍仅约 20 次 FLOP"),
    ("工程", "1 Hz 自降频让差分降噪 50 倍；激励门控（|δ|>3° 且 |r|/|dr| 或条件）是 PE 的在线实现；"
             "P 保底防塌陷、锚点管理防毛刺毒化、valid 物理门拒报不合理估计"),
    ("验证", "Z 形试验 175 s 收官：Khat +3.4%、That +1.6%；三船型误差均远优于 ±25% 漂移门；"
             "valid 13 s 置位建基线；测试 9/9 + 2/2 + 17/17 全部通过"),
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
para(tf, [("RLS 的价值不在“辨得多准”，而在 在线、低开销、带多重工程防御 地给出可信 K/T —— 让极点配置基线始终站在当前船况上", 16, True, WHITE)],
     first=True, align=PP_ALIGN.CENTER, sa=0, ls=1.15)
tf = tb(s, MARGIN, 5.86, 12.23, 1.0)
para(tf, [("谢谢，欢迎讨论", 18, True, BLUE)], first=True, align=PP_ALIGN.CENTER, sa=6)
para(tf, [("参考：.scratch/rls_report/math_derivation.md · code_facts.md · control/include/nomoto_identifier.hpp · docs/工程总体分析与ESC方案原理.md §2.3", 14, False, MUTED)],
     align=PP_ALIGN.CENTER, sa=0)

# ============================================================
# 统一后处理：降级微软雅黑未覆盖的 Unicode 字符（Windows 豆腐块修复）
#   上标 ᵀ²³⁴¹⁻ᵃ → 对应普通字符 + rPr baseline=30000（真上标）
#   下标 ₀₁       → 对应普通字符 + rPr baseline=-25000（真下标）
#   ṙ → dr/dt（“·ṙ” 系数形式 → “·(dr/dt)” 带括号）；−（U+2212）→ ASCII '-'
# ============================================================
SUP_MAP = {"ᵀ": "T", "²": "2", "³": "3", "⁴": "4", "¹": "1", "⁻": "-", "ᵃ": "a"}
SUB_MAP = {"₀": "0", "₁": "1"}


def _plain_fix(text):
    """无需特殊格式的纯文本替换：ṙ → dr/dt，U+2212 → '-'。"""
    return text.replace("·ṙ", "·(dr/dt)").replace("ṙ", "dr/dt").replace("−", "-")


def _split_segments(text):
    """按字符类别把文本拆成 [text, kind] 段（kind ∈ None/'sup'/'sub'），相邻同类合并。"""
    segs = []
    for ch in text:
        if ch in SUP_MAP:
            kind, out = "sup", SUP_MAP[ch]
        elif ch in SUB_MAP:
            kind, out = "sub", SUB_MAP[ch]
        else:
            kind, out = None, ch
        if segs and segs[-1][1] == kind:
            segs[-1][0] += out
        else:
            segs.append([out, kind])
    return segs


def _fix_run(run):
    """就地修复一个 run：无上下标时只改文本；含上下标时拆分为连续 runs，
    通过深拷贝原 rPr 完整保留字号/粗体/颜色/字体名。"""
    text = run.text
    fixed = _plain_fix(text)
    if not any(ch in SUP_MAP or ch in SUB_MAP for ch in fixed):
        if fixed != text:
            run.text = fixed
        return
    segs = _split_segments(fixed)
    r_el = run._r
    parent = r_el.getparent()
    idx = list(parent).index(r_el)
    rPr = r_el.find(qn("a:rPr"))
    for i, (seg_text, kind) in enumerate(segs):
        nr = etree.Element(qn("a:r"))
        if rPr is not None:
            nr.append(copy.deepcopy(rPr))
        if kind is not None:
            npr = nr.find(qn("a:rPr"))
            if npr is None:
                npr = etree.SubElement(nr, qn("a:rPr"))
            npr.set("baseline", "30000" if kind == "sup" else "-25000")
        t = etree.SubElement(nr, qn("a:t"))
        t.text = seg_text
        parent.insert(idx + i, nr)
    parent.remove(r_el)


def _postprocess_deck():
    for slide in prs.slides:
        for shape in slide.shapes:
            if not shape.has_text_frame:
                continue
            for p in shape.text_frame.paragraphs:
                for r in list(p.runs):
                    _fix_run(r)


_postprocess_deck()

os.makedirs(OUT_DIR, exist_ok=True)
prs.save(OUT)
print(f"saved: {OUT}, slides={len(prs.slides._sldIdLst)}")
