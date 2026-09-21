#!/usr/bin/env python3
"""将 Markdown 文件转换为 PDF（markdown-it + KaTeX + mermaid + Chrome Headless）"""

import subprocess
import sys
import os
import re
import html as html_mod
import tempfile
from pathlib import Path
from urllib.parse import unquote, urlsplit
from markdown_it import MarkdownIt

# ============ 常量 ============
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PDF_DIR = os.path.join(BASE_DIR, "pdf_output")

FONT_FAMILY = "'Noto Sans CJK SC', 'Noto Serif CJK SC', 'AR PL UKai CN', sans-serif"
MONO_FONT = "'Noto Sans Mono CJK SC', 'DejaVu Sans Mono', monospace"

CDN_KATEX_CSS = "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css"
CDN_KATEX_JS = "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js"
CDN_KATEX_AUTORENDER = "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/contrib/auto-render.min.js"
CDN_MERMAID_JS = "https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js"


def _find_asset_dir():
    """优先使用本地 KaTeX/Mermaid 资源（离线环境），否则回退 CDN。"""
    candidates = [
        os.environ.get("MD2PDF_ASSET_DIR", ""),
        "/home/jim/.cursor/extensions/allwin-antony.md-viewer-1.1.4-universal/resources",
        "/home/jim/.vscode/extensions/yzhang.markdown-all-in-one-3.6.3/resources",
        "/home/jim/.trae-cn/extensions/yzhang.markdown-all-in-one-3.6.3/resources",
    ]
    for base in candidates:
        if (base and os.path.isdir(os.path.join(base, "katex"))
                and os.path.isdir(os.path.join(base, "mermaid"))):
            return base
    return None


_ASSET_DIR = _find_asset_dir()
if _ASSET_DIR:
    def _asset_url(rel):
        return "file://" + os.path.abspath(os.path.join(_ASSET_DIR, rel))
    KATEX_CSS = _asset_url("katex/katex.min.css")
    KATEX_JS = _asset_url("katex/katex.min.js")
    KATEX_AUTORENDER = _asset_url("katex/auto-render.min.js")
    MERMAID_JS = _asset_url("mermaid/mermaid.min.js")
else:
    KATEX_CSS, KATEX_JS, KATEX_AUTORENDER, MERMAID_JS = (
        CDN_KATEX_CSS, CDN_KATEX_JS, CDN_KATEX_AUTORENDER, CDN_MERMAID_JS)

CSS = f"""
* {{ box-sizing: border-box; }}
body {{
    font-family: {FONT_FAMILY};
    font-size: 11pt;
    line-height: 1.75;
    color: #1a1a1a;
    max-width: 820px;
    margin: 0 auto;
    padding: 0 20px;
}}
h1 {{
    font-size: 21pt;
    border-bottom: 2px solid #2c5aa0;
    padding-bottom: 10px;
    margin-top: 30px;
    margin-bottom: 16px;
    color: #1a1a1a;
}}
h2 {{
    font-size: 16pt;
    border-bottom: 1px solid #b0b0b0;
    padding-bottom: 6px;
    margin-top: 26px;
    margin-bottom: 12px;
}}
h3 {{ font-size: 13.5pt; margin-top: 22px; margin-bottom: 10px; }}
h4 {{ font-size: 12pt; margin-top: 18px; }}
h5, h6 {{ font-size: 11pt; }}
p {{ margin: 8px 0; text-align: justify; }}
strong {{ color: #111; }}

blockquote {{
    border-left: 4px solid #4a90d9;
    padding: 10px 16px;
    margin: 14px 0;
    background: #f0f6ff;
    color: #333;
    border-radius: 0 4px 4px 0;
}}
blockquote p {{ margin: 4px 0; }}

pre {{
    background: #f6f8fa;
    border: 1px solid #e0e0e0;
    border-radius: 6px;
    padding: 12px 16px;
    overflow-x: auto;
    font-family: {MONO_FONT};
    font-size: 9.5pt;
    line-height: 1.5;
    white-space: pre-wrap;
    word-wrap: break-word;
    page-break-inside: avoid;
}}
code {{
    font-family: {MONO_FONT};
    font-size: 9.5pt;
    background: #f0f0f0;
    padding: 1px 5px;
    border-radius: 3px;
}}
pre code {{
    background: none;
    padding: 0;
}}

table {{
    border-collapse: collapse;
    width: 100%;
    margin: 14px 0;
    font-size: 10.5pt;
    page-break-inside: auto;
}}
th, td {{
    border: 1px solid #c8c8c8;
    padding: 7px 11px;
    text-align: left;
    vertical-align: top;
    overflow-wrap: anywhere;
}}
th {{ background: #e8f0fe; font-weight: bold; }}
tr:nth-child(even) {{ background: #fafafa; }}

img {{ max-width: 100%; height: auto; }}
hr {{ border: none; border-top: 1px solid #d0d0d0; margin: 22px 0; }}
a {{ color: #1a73e8; text-decoration: none; }}
a:hover {{ text-decoration: underline; }}
ul, ol {{ padding-left: 26px; margin: 8px 0; }}
li {{ margin: 4px 0; }}

/* 目录区块 */
h1#目录, h2#目录 {{ border: none; color: #333; }}

/* KaTeX 公式 */
.katex {{ font-size: 1.05em; }}
.katex-display {{
    margin: 14px 0;
    padding: 10px 0;
    overflow-x: auto;
    overflow-y: hidden;
}}

/* mermaid 图 */
pre.mermaid-block {{
    background: #ffffff;
    border: none;
    padding: 10px 0;
    text-align: center;
    overflow: visible;
    white-space: normal;
}}
pre.mermaid-block svg {{ max-width: 100%; height: auto; }}
pre.mermaid-block p {{ text-align: center; }}

/* 页面设置 */
@page {{
    size: A4;
    margin: 18mm 16mm 18mm 16mm;
}}

h2 {{ page-break-after: avoid; }}
h3, h4, h5, h6 {{ page-break-after: avoid; }}
tr, img {{ page-break-inside: avoid; }}
thead {{ display: table-header-group; }}
@media print {{
    pre {{ page-break-inside: auto; }}
    .katex-display {{ overflow: visible; }}
}}
"""


def github_slug(text: str) -> str:
    """生成 GitHub 风格锚点 id：转小写、去标点、空格转横线（保留中文）。"""
    children = MarkdownIt("commonmark").parseInline(text)[0].children or []
    text = "".join(t.content for t in children if t.type in ("text", "code_inline", "image"))
    text = text.lower()
    text = re.sub(r"[^\w\u4e00-\u9fff\s-]", "", text, flags=re.UNICODE)
    return re.sub(r"\s", "-", text)


def build_md_parser() -> MarkdownIt:
    md = MarkdownIt("gfm-like", {"html": True})

    # 给标题添加 GitHub 风格锚点 id
    def heading_open(tokens, idx, options, env):
        token = tokens[idx]
        inline = tokens[idx + 1] if idx + 1 < len(tokens) and tokens[idx + 1].type == "inline" else None
        text = inline.content if inline else ""
        for placeholder, original in env.get("math_placeholders", {}).items():
            text = text.replace(placeholder, original)
        base = github_slug(text)
        used = env.setdefault("heading_ids", set())
        slug, count = base, 0
        while slug in used:
            count += 1
            slug = f"{base}-{count}"
        used.add(slug)
        return f'<{token.tag} id="{slug}">'

    md.renderer.rules["heading_open"] = heading_open
    return md


def protect_math(md_text: str):
    r"""把 $...$/$$...$$ 数学块替换为占位符，避免 markdown-it 转义数学反斜杠。

    markdown-it 会把 `\\`、`\,`、`\;` 等按 CommonMark 转义处理（如 `\\`→`\`、
    `\,`→`,`），导致 KaTeX 公式被破坏。这里先按原样取出数学内容、用占位符渲染
    Markdown，渲染后再还原，保证 KaTeX 收到未经转义的原始公式。
    """
    lines = md_text.splitlines(keepends=True)
    masked_code, placeholders = {}, {}
    prefix = "@@DOCMATH"
    while prefix in md_text:
        prefix += "X"

    def save_code(value):
        token = f"{prefix}CODE{len(masked_code)}@@"
        masked_code[token] = value
        return token

    # Parser maps handle tilde fences, longer fences and nested code blocks.
    for token in reversed(MarkdownIt("commonmark").parse(md_text)):
        if token.type in ("fence", "code_block") and token.map:
            start, end = token.map
            lines[start:end] = [save_code("".join(lines[start:end]))]
    masked = "".join(lines)
    masked = re.sub(r"(`+)([\s\S]*?)\1(?!`)", lambda m: save_code(m[0]), masked)

    def save_math(match):
        token = f"{prefix}{len(placeholders)}@@"
        placeholders[token] = match[0]
        return token

    pattern = (r"(?<!\\)\$\$[\s\S]+?(?<!\\)\$\$"
               r"|(?<![\\$])\$(?![$\s])(?:\\.|[^$\\\n])+?(?<![\\\s])\$(?!\$)"
               r"|(?<!\\)\\\([\s\S]+?\\\)"
               r"|(?<!\\)\\\[[\s\S]+?\\\]")
    masked = re.sub(pattern, save_math, masked)
    for token, original in masked_code.items():
        masked = masked.replace(token, original)
    return masked, placeholders


def restore_math(html_body: str, placeholders: dict) -> str:
    """Restore TeX as HTML text, never as markup (notably <, > and &)."""
    for token, original in placeholders.items():
        html_body = html_body.replace(token, html_mod.escape(original, quote=False))
    return html_body


def fix_relative_links(html_body: str, md_dir: str) -> str:
    """将相对路径图片转换为 file:// 绝对路径，保留中文和 URL 编码。"""
    def repl_img(m):
        attrs = m.group(1)
        def repl_src(sm):
            url = html_mod.unescape(sm.group(1))
            parsed = urlsplit(url)
            if parsed.scheme or url.startswith("//"):
                return sm.group(0)
            uri = (Path(md_dir) / unquote(parsed.path)).resolve().as_uri()
            if parsed.fragment:
                uri += "#" + parsed.fragment
            return f'src="{html_mod.escape(uri, quote=True)}"'
        attrs = re.sub(r'src="([^"]*)"', repl_src, attrs)
        return "<img " + attrs + ">"

    html_body = re.sub(r"<img\s+([^>]*?)/?>", repl_img, html_body)
    return html_body


def md_to_html(md_path: str) -> str:
    with open(md_path, "r", encoding="utf-8") as f:
        md_content = f.read()

    md_content, math_placeholders = protect_math(md_content)
    md = build_md_parser()
    body = md.render(md_content, {"math_placeholders": math_placeholders})
    body = restore_math(body, math_placeholders)
    md_dir = os.path.dirname(md_path)
    body = fix_relative_links(body, md_dir)

    title = os.path.splitext(os.path.basename(md_path))[0]

    html = f"""<!DOCTYPE html>
<html lang="zh-CN" data-render-status="pending">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html_mod.escape(title)}</title>
<link rel="stylesheet" href="{KATEX_CSS}">
<style>{CSS}</style>
</head>
<body>
{body}
<script src="{KATEX_JS}"></script>
<script src="{KATEX_AUTORENDER}"></script>
<script src="{MERMAID_JS}"></script>
<script>
async function renderAll() {{
    try {{
        // 数学公式包括表格单元格；代码块中的美元符号不作为公式。
        if ({str(bool(math_placeholders)).lower()} && !window.renderMathInElement) {{
            throw new Error('KaTeX resources unavailable');
        }}
        if (window.renderMathInElement) {{
            renderMathInElement(document.body, {{
                delimiters: [
                    {{ left: '$$', right: '$$', display: true }},
                    {{ left: '$', right: '$', display: false }},
                    {{ left: '\\\\(', right: '\\\\)', display: false }},
                    {{ left: '\\\\[', right: '\\\\]', display: true }}
                ],
                throwOnError: false,
                strict: 'ignore'
            }});
        }}
        // 渲染 mermaid 图
        const diagrams = document.querySelectorAll('pre code.language-mermaid');
        if (diagrams.length && !window.mermaid) throw new Error('Mermaid resources unavailable');
        if (diagrams.length) {{
            document.querySelectorAll('pre code.language-mermaid').forEach(function (el) {{
                var pre = el.closest('pre');
                if (pre) pre.classList.add('mermaid-block');
            }});
            mermaid.initialize({{
                startOnLoad: false,
                theme: 'default',
                securityLevel: 'loose',
                fontFamily: {FONT_FAMILY!r}
            }});
            await mermaid.run({{ querySelector: 'pre code.language-mermaid' }});
        }}
        await document.fonts.ready;
        await Promise.all(Array.from(document.images, img => img.complete ? Promise.resolve() :
            new Promise(resolve => {{ img.onload = resolve; img.onerror = resolve; }})));
        if (document.querySelector('.katex-error') ||
            Array.from(document.images).some(img => !img.naturalWidth)) {{
            throw new Error('Invalid formula or missing image');
        }}
        // Shrink only oversized display equations, preserving complete expressions.
        document.querySelectorAll('.katex-display').forEach(el => {{
            const equation = el.querySelector('.katex');
            if (equation && equation.scrollWidth > el.clientWidth) {{
                equation.style.fontSize = (100 * el.clientWidth / equation.scrollWidth) + '%';
            }}
        }});
        document.documentElement.dataset.renderStatus = 'ready';
    }} catch (e) {{
        document.documentElement.dataset.renderStatus = 'error';
        document.documentElement.dataset.renderError = String(e);
        console.error('render error:', e);
    }}
}}
renderAll();
</script>
</body>
</html>"""
    return html


def html_to_pdf(html_content: str, pdf_path: str):
    """Only replace the previous PDF after resources and diagrams finish rendering."""
    destination = Path(pdf_path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".md2pdf-", dir=destination.parent) as temp:
        tmp_html, tmp_pdf = Path(temp) / "document.html", Path(temp) / "document.pdf"
        tmp_html.write_text(html_content, encoding="utf-8")
        cmd = [
            "google-chrome", "--headless=new", "--disable-gpu", "--no-sandbox",
            "--disable-dev-shm-usage", "--no-first-run", "--no-default-browser-check",
            "--allow-file-access-from-files", f"--user-data-dir={temp}/profile",
            f"--print-to-pdf={tmp_pdf}", "--no-pdf-header-footer",
            "--virtual-time-budget=60000", "--dump-dom", tmp_html.as_uri(),
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        except subprocess.TimeoutExpired:
            print("  Chrome 转换超时；保留原 PDF")
            return False
        if result.returncode != 0 or not tmp_pdf.exists():
            print(f"  Chrome 错误；保留原 PDF: {result.stderr[:500]}")
            return False
        if 'data-render-status="ready"' not in result.stdout:
            error = re.search(r'data-render-error="([^"]*)"', result.stdout)
            print("  渲染未完成；保留原 PDF:", html_mod.unescape(error[1]) if error else "pending")
            return False
        if not tmp_pdf.read_bytes().startswith(b"%PDF-"):
            print("  PDF 输出无效；保留原 PDF")
            return False
        os.replace(tmp_pdf, destination)
    return True


def main():
    md_files = [
        "自动舵定位与控制设计文档.md",
        "自动舵设计技术可行性分析文档.md",
        "软件设计与架构文档.md",
        "control/控制算法开发文档.md",
        "control/控制算法代码说明文档.md",
        "control/tools/README.md",
        "control/ui/README.md",
    ]

    # 支持命令行传入指定 md 文件（相对 BASE_DIR 的路径或绝对路径）
    args = sys.argv[1:]
    if args == ["--existing"]:
        # Refresh matching Markdown exports only; never rewrite source papers.
        candidates = subprocess.run(["rg", "--files", "-g", "*.md"], cwd=BASE_DIR,
                                    capture_output=True, text=True, check=True)
        md_files = [name for name in sorted(candidates.stdout.splitlines())
                    if (Path(PDF_DIR) / (name[:-3].replace("/", "_") + ".pdf")).exists()]
    elif args:
        md_files = args

    os.makedirs(PDF_DIR, exist_ok=True)
    success = 0
    fail = 0

    for md_file in md_files:
        md_path = os.path.join(BASE_DIR, md_file)
        if not os.path.exists(md_path):
            print(f"[跳过] 文件不存在: {md_file}")
            fail += 1
            continue

        relative = os.path.relpath(md_path, BASE_DIR)
        pdf_stem = relative.replace("/", "_").replace("\\", "_")
        pdf_name = os.path.splitext(pdf_stem)[0] + ".pdf"
        pdf_path = os.path.join(PDF_DIR, pdf_name)

        print(f"[转换] {md_file} → pdf_output/{pdf_name}")
        try:
            html_content = md_to_html(md_path)
            if html_to_pdf(html_content, pdf_path):
                file_size = os.path.getsize(pdf_path)
                print(f"  [OK] {pdf_name} ({file_size / 1024:.1f} KB)")
                success += 1
            else:
                print(f"  [失败] {pdf_name}")
                fail += 1
        except Exception as e:
            print(f"  [错误] {e}")
            fail += 1

    print(f"\n完成: 成功 {success} 个, 失败 {fail} 个")
    print(f"PDF 输出目录: {PDF_DIR}")
    return bool(fail)


if __name__ == "__main__":
    sys.exit(main())
