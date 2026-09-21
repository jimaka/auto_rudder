#!/usr/bin/env python3
"""Render the architecture Markdown as an offline, printable HTML document."""
from html import escape
from pathlib import Path

from markdown_it import MarkdownIt

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "项目架构与全函数调用说明.md"
TARGET = HERE / "项目架构说明.html"


def main():
    parser = MarkdownIt("commonmark", {"html": False}).enable("table")
    tokens = parser.parse(SOURCE.read_text(encoding="utf-8"))
    toc = []
    for i, token in enumerate(tokens):
        if token.type == "heading_open" and token.tag == "h2":
            uid = f"section-{len(toc) + 1}"
            token.attrSet("id", uid)
            toc.append((uid, tokens[i + 1].content))

    def links(items):
        for token in items:
            for attribute in ("src", "href"):
                value = token.attrGet(attribute)
                if value and not value.startswith(("#", "https:", "http:", "mailto:", "data:", "/")):
                    token.attrSet(attribute, "../" + value)
            if token.children:
                links(token.children)
    links(tokens)
    body = parser.renderer.render(tokens, parser.options, {})
    nav = "".join(f'<a href="#{uid}">{escape(label)}</a>' for uid, label in toc)
    page = '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>自动舵项目架构与全函数调用说明</title>
<style>*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:#f0f5f8;color:#243c4c;font:16px/1.85 system-ui,"Noto Sans CJK JP",sans-serif}aside{position:fixed;inset:0 auto 0 0;width:270px;padding:26px 18px;overflow:auto;background:#13334c;color:white}aside b{display:block;font-size:23px;margin-bottom:15px}aside a{display:block;color:#c5e1ee;padding:6px 8px;font-size:13px;text-decoration:none;border-radius:4px}aside a:hover{background:#2c526b;color:white}main{margin:0 24px 0 295px;max-width:1120px;background:white;padding:40px 44px;box-shadow:0 3px 24px #16324a0d}h1{font-size:32px;color:#123852;line-height:1.45}h2{font-size:24px;color:#185477;border-bottom:2px solid #dceaf2;padding:20px 0 10px;scroll-margin-top:10px}h3{font-size:19px;color:#2d566e;margin-top:30px}a{color:#0075a3}p{margin:14px 0}img{display:block;width:100%;height:auto;border:1px solid #e3ebf1;border-radius:8px;margin:22px 0}table{border-collapse:collapse;width:100%;font-size:14px;margin:20px 0}td,th{border:1px solid #d5e1e8;padding:9px 11px;text-align:left;vertical-align:top}th{background:#eaf3f8}tr:nth-child(even) td{background:#f9fbfc}code{background:#edf3f7;padding:2px 4px;border-radius:3px;font-size:.9em;overflow-wrap:anywhere}pre{overflow:auto;background:#132f45;color:#d6ebf5;padding:20px;border-radius:7px;font:13px/1.7 monospace}pre code{background:transparent;color:inherit;padding:0}blockquote{margin:20px 0;padding:5px 20px;border-left:4px solid #d9a148;background:#fff8e8}li{margin:8px 0}.toolbar{font-size:14px;background:#eaf5fa;padding:15px;border-radius:6px}@media(max-width:950px){aside{position:static;width:auto;max-height:260px}main{margin:0;padding:22px;max-width:none}table{font-size:12px}h1{font-size:26px}}@media print{body{background:white;font-size:10pt}aside,.toolbar{display:none}main{margin:0;max-width:none;padding:0;box-shadow:none}h2{break-after:avoid}h3{break-after:avoid}img{break-inside:avoid;max-height:230mm;object-fit:contain}tr{break-inside:avoid}pre{white-space:pre-wrap}a{color:inherit;text-decoration:none}@page{size:A4;margin:16mm}}
</style></head><body><aside><b>自动舵 · 架构说明</b>NAV</aside><main><p class="toolbar"><a href="../项目架构与全函数调用说明.md">Markdown 原文</a> · <a href="call_atlas.html">全函数调用图谱</a> · <a href="全函数索引.md">全函数索引</a> · 浏览器打印可保存 PDF</p>BODY</main></body></html>'''
    TARGET.write_text(page.replace("NAV", nav).replace("BODY", body), encoding="utf-8")
    print(f"Rendered {len(toc)} sections: {TARGET}")


if __name__ == "__main__":
    main()
