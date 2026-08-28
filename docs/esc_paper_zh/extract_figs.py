"""从 esc_survey.pdf 中提取每张插图区域并渲染为 PNG。

原理：Elsevier 单栏排版中，图题（"Fig. N. ..."）位于插图正下方。
对每一页：
1. 找出所有以 "Fig. N." 开头的文本块（图题）。
2. 收集页面上的图形内容：嵌入栅格图的矩形 + 矢量绘图聚类。
3. 把图形聚类分配给其下方最近的图题，取并集后按区域渲染。
没有图题认领的图形（跨页图的前续面板）单独导出为 loose 图。
"""
import re
import pymupdf

PDF = "esc_survey.pdf"
OUT = "images"
MARGIN = 8.0  # 聚类合并/吸附的容差 (pt)
DPI = 170

doc = pymupdf.open(PDF)


def union(a, b):
    r = pymupdf.Rect(a)
    r.include_rect(b)
    return r


def near_or_overlap(a, b, m=MARGIN):
    return not pymupdf.Rect(a.x0 - m, a.y0 - m, a.x1 + m, a.y1 + m).intersects(b) is False \
        or pymupdf.Rect(a.x0 - m, a.y0 - m, a.x1 + m, a.y1 + m).intersects(b)


def clusters_overlap(a, b, m=MARGIN):
    """两个矩形（扩 m 后）是否相交或垂直方向紧邻。"""
    ea = pymupdf.Rect(a.x0 - m, a.y0 - m, a.x1 + m, a.y1 + m)
    return ea.intersects(b)


mapping = []   # (fignum, page, filename)
loose = []

cap_re = re.compile(r"^\s*Fig\.?\s*(\d+)\s*[.(]?", re.I)

for pno in range(1, doc.page_count):  # 跳过第 1 页（LANL 封面）
    page = doc[pno]
    blocks = page.get_text("blocks")
    captions = []  # (fignum, rect)
    for b in blocks:
        text = b[4].strip()
        m = cap_re.match(text)
        if m:
            captions.append((int(m.group(1)), pymupdf.Rect(b[:4])))

    # 收集图形矩形
    grects = []
    for img in page.get_images(full=True):
        xref = img[0]
        for r in page.get_image_rects(xref):
            if r.width > 40 and r.height > 40:  # 过滤小图标/装饰
                grects.append(r)
    try:
        for r in page.cluster_drawings():
            if r.width > 40 and r.height > 40:
                grects.append(r)
    except Exception:
        pass

    # 合并相交/紧邻的矩形为聚类
    merged = True
    while merged:
        merged = False
        out = []
        used = [False] * len(grects)
        for i, r in enumerate(grects):
            if used[i]:
                continue
            cur = pymupdf.Rect(r)
            used[i] = True
            for j in range(i + 1, len(grects)):
                if not used[j] and clusters_overlap(cur, grects[j]):
                    cur = union(cur, grects[j])
                    used[j] = True
                    merged = True
            out.append(cur)
        grects = out

    # 分配：每个聚类归到其下方最近（且距离 < 400pt）的图题
    assigned = {i: [] for i in range(len(captions))}
    leftovers = []
    for r in grects:
        best, bestd = -1, 1e9
        for i, (num, cr) in enumerate(captions):
            d = cr.y0 - r.y1
            if -5 <= d < bestd:
                best, bestd = i, d
        if best >= 0 and bestd < 400:
            assigned[best].append(r)
        else:
            leftovers.append(r)

    for i, (num, cr) in enumerate(captions):
        if not assigned[i]:
            continue
        region = assigned[i][0]
        for r in assigned[i][1:]:
            region = union(region, r)
        region = region & page.rect
        if region.is_empty:
            continue
        fname = f"fig{num:02d}_p{pno + 1:02d}.png"
        page.get_pixmap(clip=region, dpi=DPI).save(f"{OUT}/{fname}")
        mapping.append((num, pno + 1, fname))

    for k, r in enumerate(leftovers):
        region = r & page.rect
        if region.is_empty:
            continue
        fname = f"loose_p{pno + 1:02d}_{k}.png"
        page.get_pixmap(clip=region, dpi=DPI).save(f"{OUT}/{fname}")
        loose.append((pno + 1, fname))

print("FIGURES:")
for num, page, fname in sorted(mapping):
    print(f"  Fig.{num}: {fname}")
print("LOOSE:")
for page, fname in loose:
    print(f"  p{page}: {fname}")
