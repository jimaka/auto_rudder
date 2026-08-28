#!/usr/bin/env python3
"""preview_ppt.py — 读取 pptx，用 PIL 渲染每页 PNG 供目检，并检测文本溢出。

字体回退链：Noto Sans CJK（中文/常用符号）→ DejaVu Sans（数学符号/组合变音符）。
近似 PowerPoint 排版：CJK 任意断行、拉丁按词断行；行高 = 字号 × 行距 × 1.2。
检测：文本超出框高 / 单词超框宽 → 打印告警并在图上画红框。

用法：.venv-ppt/bin/python .scratch/esc_report/preview_ppt.py [pptx] [outdir]
"""
import io
import os
import sys

from PIL import Image, ImageDraw, ImageFont
from pptx import Presentation
from pptx.enum.shapes import MSO_SHAPE_TYPE
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.util import Emu

DPI = 110  # 渲染分辨率 px/in
NOTO_R = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
NOTO_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
DEJA_R = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
DEJA_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

_font_cache = {}
_glyph_sig_cache = {}


def _ink(font, ch):
    """渲染字形像素签名（与 .notdef 比对；bbox 比对会与方框轮廓误判）。"""
    key = (id(font), ch)
    sig = _glyph_sig_cache.get(key)
    if sig is None:
        img = Image.new("L", (font.size + 8, font.size + 8), 0)
        ImageDraw.Draw(img).text((4, 2), ch, font=font, fill=255)
        sig = img.tobytes()
        _glyph_sig_cache[key] = sig
    return sig


def _font(px, bold):
    key = (px, bold)
    if key not in _font_cache:
        nf = ImageFont.truetype(NOTO_B if bold else NOTO_R, px, index=2)
        df = ImageFont.truetype(DEJA_B if bold else DEJA_R, px)
        # 缺失字形判定：像素签名与 .notdef 一致即视为缺字
        _font_cache[key] = (nf, df, _ink(nf, "\uFFFF"), _ink(df, "\uFFFF"))
    return _font_cache[key]


def _has_glyph(font, miss, ch):
    return _ink(font, ch) != miss



def _w(px, bold, ch):
    nf, df, miss_n, miss_d = _font(px, bold)
    if _is_cjk(ch):
        # CJK 字符优先 Noto
        if _has_glyph(nf, miss_n, ch):
            return nf.getlength(ch), nf
        return df.getlength(ch), df
    # 拉丁/符号优先 DejaVu（Noto CJK 的部分拉丁扩展字形渲染异常，如 ć、–）
    if _has_glyph(df, miss_d, ch):
        return df.getlength(ch), df
    return nf.getlength(ch), nf


def _is_cjk(ch):
    o = ord(ch)
    return (0x2E80 <= o <= 0x9FFF or 0x3000 <= o <= 0x303F
            or 0xFF00 <= o <= 0xFFEF or 0xF900 <= o <= 0xFAFF)


def wrap_runs(runs, box_w_px):
    """runs: [(text, px, bold, color)] → lines: [[(chunk, px, bold, font, w, color)], ...]

    token 级断行（CJK 逐字、拉丁成词）；token 内按字体回退切成 chunk 绘制。
    """
    tokens = []
    for text, px, bold, color in runs:
        buf = ""
        for ch in text:
            k = "cjk" if _is_cjk(ch) else ("space" if ch == " " else "lat")
            if k != "lat":
                if buf:
                    tokens.append((buf, px, bold, color))
                    buf = ""
                tokens.append((ch, px, bold, color))
            else:
                buf += ch
        if buf:
            tokens.append((buf, px, bold, color))
    lines, cur, cur_w = [], [], 0.0
    max_tok = 0.0
    for tok, px, bold, color in tokens:
        # 按字形回退把 token 切成 chunk（相邻同字体字符合并）
        chunks, cbuf, cfont, cw_sum = [], "", None, 0.0
        for ch in tok:
            cw, f = _w(px, bold, ch)
            if f is not cfont and cbuf:
                chunks.append((cbuf, px, bold, cfont, cw_sum, color))
                cbuf, cw_sum = "", 0.0
            cfont = f
            cbuf += ch
            cw_sum += cw
        if cbuf:
            chunks.append((cbuf, px, bold, cfont, cw_sum, color))
        w = sum(c[4] for c in chunks)
        if tok.strip():
            max_tok = max(max_tok, w)
        if tok == " " and cur_w == 0:
            continue
        if cur and cur_w + w > box_w_px and tok.strip():
            lines.append(cur)
            cur, cur_w = [], 0.0
            if tok == " ":
                continue
        cur.extend(chunks)
        cur_w += w
    if cur:
        lines.append(cur)
    return lines, max_tok


def render(pptx_path, outdir):
    prs = Presentation(pptx_path)
    sw_in = Emu(prs.slide_width).inches
    sh_in = Emu(prs.slide_height).inches
    W, H = int(sw_in * DPI), int(sh_in * DPI)
    os.makedirs(outdir, exist_ok=True)
    warnings = []
    for idx, slide in enumerate(prs.slides, 1):
        img = Image.new("RGB", (W, H), "white")
        dr = ImageDraw.Draw(img)
        for shp in slide.shapes:
            x = Emu(shp.left).inches * DPI
            y = Emu(shp.top).inches * DPI
            w = Emu(shp.width).inches * DPI
            h = Emu(shp.height).inches * DPI
            st = shp.shape_type
            if st == MSO_SHAPE_TYPE.PICTURE:
                pim = Image.open(io.BytesIO(shp.image.blob)).convert("RGB")
                pim = pim.resize((max(1, int(w)), max(1, int(h))))
                img.paste(pim, (int(x), int(y)))
                continue
            if st == MSO_SHAPE_TYPE.AUTO_SHAPE:
                try:
                    fill = shp.fill
                    if fill.type is not None and fill.fore_color.type is not None:
                        dr.rectangle([x, y, x + w, y + h],
                                     fill="#" + str(fill.fore_color.rgb))
                except Exception:
                    pass
                try:
                    if shp.line.fill.type is not None:
                        dr.rectangle([x, y, x + w, y + h],
                                     outline="#" + str(shp.line.color.rgb), width=2)
                except Exception:
                    pass
            if not shp.has_text_frame:
                continue
            tf = shp.text_frame
            if not tf.word_wrap:
                # 不换行的装饰性小框（如箭头）：只做绘制，不做溢出检测
                do_check = False
            else:
                do_check = True
            anchor = tf.vertical_anchor
            # 先排版所有段落，求总高
            blocks = []
            total_h = 0.0
            for p in tf.paragraphs:
                runs = []
                for r in p.runs:
                    sz = r.font.size.pt if r.font.size else 18
                    try:
                        color = "#" + str(r.font.color.rgb)
                    except Exception:
                        color = "#262626"
                    runs.append((r.text, int(sz * DPI / 72), bool(r.font.bold), color))
                if not runs:
                    continue
                ls = p.line_spacing if isinstance(p.line_spacing, float) else 1.0
                sa = (p.space_after.pt if p.space_after else 0) * DPI / 72
                sb = (p.space_before.pt if p.space_before else 0) * DPI / 72
                lines, max_tok = wrap_runs(runs, w - 0.08 * DPI)
                # 每行用各自最大字号
                hs = [max(px for _, px, _, _, _, _ in ln) * 1.2 * ls for ln in lines]
                ph = sum(hs) + sa + sb
                blocks.append((p, lines, hs, sa, sb))
                total_h += ph
                if do_check and max_tok > w - 0.08 * DPI:
                    warnings.append(f"slide {idx}: 单词超框宽 shape@({x:.0f},{y:.0f})")
            # 垂直锚点
            ml = Emu(tf.margin_left).inches * DPI if tf.margin_left else 0
            mt = Emu(tf.margin_top).inches * DPI if tf.margin_top else 0
            if anchor == MSO_ANCHOR.MIDDLE:
                cy = y + mt + max(0, (h - total_h) / 2)
            else:
                cy = y + mt
            if do_check and total_h > h + 4:
                warnings.append(
                    f"slide {idx}: 文本超高 {total_h:.0f}px > 框 {h:.0f}px @({x/DPI:.2f},{y/DPI:.2f})in")
                dr.rectangle([x, y, x + w, y + h], outline="red", width=4)
            for p, lines, hs, sa, sb in blocks:
                cy += sb
                for ln, lh in zip(lines, hs):
                    line_w = sum(tw for _, _, _, _, tw, _ in ln)
                    if p.alignment == PP_ALIGN.CENTER:
                        cx = x + ml + max(0, (w - line_w) / 2)
                    elif p.alignment == PP_ALIGN.RIGHT:
                        cx = x + w - line_w - 0.04 * DPI
                    else:
                        cx = x + ml
                    base = cy + lh * 0.78
                    for tok, px, bold, f, tw, color in ln:
                        dr.text((cx, base), tok, font=f, fill=color, anchor="ls")
                        cx += tw
                    cy += lh
                cy += sa
        out = os.path.join(outdir, f"slide_{idx:02d}.png")
        img.save(out)
    return warnings


if __name__ == "__main__":
    pptx_path = sys.argv[1] if len(sys.argv) > 1 else "ppt_output/ESC方案汇报.pptx"
    outdir = sys.argv[2] if len(sys.argv) > 2 else "/tmp/ppt_preview"
    warns = render(pptx_path, outdir)
    print(f"rendered to {outdir}")
    if warns:
        print("WARNINGS:")
        for w_ in warns:
            print(" -", w_)
    else:
        print("no overflow warnings")
