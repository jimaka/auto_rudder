#!/usr/bin/env python3
"""将 Markdown 文件转换为 PDF（markdown-it + KaTeX + mermaid + Chrome Headless）"""

import subprocess
import sys
import os
import re
import html as html_mod
from urllib.parse import urljoin
from markdown_it import MarkdownIt

# ============ 常量 ============
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PDF_DIR = os.path.join(BASE_DIR, "pdf_output")

FONT_FAMILY = "'Noto Sans CJK SC', 'Noto Serif CJK SC', 'AR PL UKai CN', sans-serif"
MONO_FONT = "'Noto Sans Mono CJK SC', 'DejaVu Sans Mono', monospace"

KATEX_CSS = "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css"
KATEX_JS = "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js"
KATEX_AUTORENDER = "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/contrib/auto-render.min.js"
MERMAID_JS = "https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js"

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

/* 页面设置 */
@page {{
    size: A4;
    margin: 18mm 16mm 18mm 16mm;
}}

h2 {{ page-break-after: avoid; }}
h3, h4, h5, h6 {{ page-break-after: avoid; }}
tr, img {{ page-break-inside: avoid; }}
"""


def github_slug(text: str) -> str:
    """生成 GitHub 风格锚点 id：转小写、去标点、空格转横线（保留中文）。"""
    text = re.sub(r'<[^>]+>', '', text)
    text = text.lower()
    text = re.sub(r"[^\w\u4e00-\u9fff\s-]", "", text, flags=re.UNICODE)
    text = re.sub(r"[\s_]+", "-", text)
    text = re.sub(r"-+", "-", text)
    return text.strip("-")


def build_md_parser() -> MarkdownIt:
    md = MarkdownIt("gfm-like", {"html": True})

    # 给标题添加 GitHub 风格锚点 id
    def heading_open(tokens, idx, options, env):
        token = tokens[idx]
        inline = tokens[idx + 1] if idx + 1 < len(tokens) and tokens[idx + 1].type == "inline" else None
        text = inline.content if inline else ""
        slug = github_slug(text)
        return f'<{token.tag} id="{slug}">'

    md.renderer.rules["heading_open"] = heading_open
    return md


def fix_relative_links(html_body: str, md_dir: str) -> str:
    """将相对路径的图片/链接转换为 file:// 绝对路径。"""
    def repl_img(m):
        attrs = m.group(1)
        def repl_src(sm):
            url = sm.group(1)
            if url.startswith(("http://", "https://", "data:", "file://")):
                return sm.group(0)
            abs_path = os.path.abspath(os.path.join(md_dir, url.split("#")[0]))
            return f'src="file://{abs_path}"'
        attrs = re.sub(r'src="([^"]*)"', repl_src, attrs)
        return "<img " + attrs + ">"

    html_body = re.sub(r"<img\s+([^>]*?)/?>", repl_img, html_body)
    return html_body


def md_to_html(md_path: str) -> str:
    with open(md_path, "r", encoding="utf-8") as f:
        md_content = f.read()

    md = build_md_parser()
    body = md.render(md_content)
    md_dir = os.path.dirname(md_path)
    body = fix_relative_links(body, md_dir)

    title = os.path.splitext(os.path.basename(md_path))[0]

    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
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
        // 渲染数学公式（忽略代码块/表格中的美元符号）
        if (window.renderMathInElement) {{
            renderMathInElement(document.body, {{
                delimiters: [
                    {{ left: '$$', right: '$$', display: true }},
                    {{ left: '$', right: '$', display: false }},
                    {{ left: '\\\\(', right: '\\\\)', display: false }},
                    {{ left: '\\\\[', right: '\\\\]', display: true }}
                ],
                throwOnError: false
            }});
        }}
        // 渲染 mermaid 图
        if (window.mermaid) {{
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
    }} catch (e) {{
        console.error('render error:', e);
    }}
}}
renderAll();
</script>
</body>
</html>"""
    return html


def html_to_pdf(html_content: str, pdf_path: str):
    tmp_html = pdf_path + ".tmp.html"
    with open(tmp_html, "w", encoding="utf-8") as f:
        f.write(html_content)

    cmd = [
        "google-chrome",
        "--headless=new",
        "--disable-gpu",
        "--no-sandbox",
        "--disable-software-rasterizer",
        "--print-to-pdf=" + os.path.abspath(pdf_path),
        "--print-to-pdf-no-header",
        "--no-pdf-header-footer",
        "--virtual-time-budget=60000",
        "file://" + os.path.abspath(tmp_html),
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if result.returncode != 0:
            print(f"  Chrome 错误: {result.stderr[:500]}")
            return False
    except subprocess.TimeoutExpired:
        print("  Chrome 转换超时")
        return False
    finally:
        if os.path.exists(tmp_html):
            os.remove(tmp_html)
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
    if args:
        md_files = args

    os.makedirs(PDF_DIR, exist_ok=True)
    success = 0
    fail = 0

    for md_file in md_files:
        md_path = os.path.join(BASE_DIR, md_file)
        if not os.path.exists(md_path):
            print(f"[跳过] 文件不存在: {md_file}")
            continue

        pdf_stem = md_file.replace("/", "_").replace("\\", "_")
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


if __name__ == "__main__":
    main()
