"""将说明文档和四幅插图打包成离线 HTML；源码链接仍指向仓库相对路径。"""
import base64
import re
from pathlib import Path
from markdown_it import MarkdownIt

root = Path(__file__).resolve().parent
source = root / '工程分析与中文说明.md'
body = MarkdownIt('commonmark', {'html': False}).enable('table').render(source.read_text())
def embed(match):
    path = root / match.group(1)
    return 'src="data:image/png;base64,' + base64.b64encode(path.read_bytes()).decode() + '"'
body = re.sub(r'src="(figures/[^\"]+\.png)"', embed, body)
css = '''body{max-width:1080px;margin:40px auto;padding:0 28px;font:16px/1.85 "Noto Sans CJK SC",sans-serif;color:#23354a;background:#fff}
h1,h2,h3{color:#183f65;line-height:1.4}h1{font-size:32px;border-bottom:4px solid #168577;padding-bottom:20px}h2{margin-top:48px;border-bottom:1px solid #d6e1eb;padding-bottom:10px}
img{display:block;max-width:100%;height:auto;margin:24px auto}table{border-collapse:collapse;width:100%;font-size:14px;margin:20px 0}th,td{border:1px solid #d7e1eb;padding:10px 12px;text-align:left}th{background:#edf4f9}tr:nth-child(even){background:#f8fafc}
code{font-family:"DejaVu Sans Mono",monospace;font-size:.88em;background:#eef3f7;padding:2px 4px;border-radius:3px}pre{padding:18px;background:#eef3f7;overflow:auto;line-height:1.55}pre code{padding:0}blockquote{margin:20px 0;padding:10px 20px;border-left:4px solid #168577;background:#f1f8f6}a{color:#2166a3}strong{color:#153e63}
@media print{body{max-width:none;margin:0;font-size:10pt}h2,h3{break-after:avoid}img, tr{break-inside:avoid}a{color:inherit;text-decoration:none}pre{white-space:pre-wrap}}'''
target = source.with_suffix('.html')
target.write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>auto_rudder 工程分析与中文说明</title><style>'+css+'</style><body>'+body+'</body></html>')
print(target)
