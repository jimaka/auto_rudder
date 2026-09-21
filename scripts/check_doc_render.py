#!/usr/bin/env python3
"""Check all documentation TeX and Mermaid using the actual browser libraries."""
import argparse
import html
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import md2pdf
from scripts.format_docs import files


def collect():
    formulas, graphs = {}, {}
    for path in files():
        source = path.read_text(encoding="utf-8")
        _, math = md2pdf.protect_math(source)
        name = str(path.relative_to(ROOT))
        for original in math.values():
            display = original.startswith(("$$", r"\["))
            width = 2 if display or original.startswith(r"\(") else 1
            tex = original[width:-width].strip()
            key = (tex, display)
            if key not in formulas:
                formulas[key] = {"tex": tex, "display": display, "sources": []}
            if name not in formulas[key]["sources"]:
                formulas[key]["sources"].append(name)
        for token in md2pdf.build_md_parser().parse(source):
            if token.type == "fence" and token.info.strip() == "mermaid":
                graphs.setdefault(token.content, {"code": token.content, "sources": []})["sources"].append(name)
    return {"formulas": list(formulas.values()), "graphs": list(graphs.values())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    payload = json.dumps(collect(), ensure_ascii=False).replace("<", r"\u003c")
    page = '''<!doctype html><meta charset="utf-8"><pre id="result">pending</pre>
<script src="KATEX"></script><script src="MERMAID"></script>
<script>const payload = PAYLOAD;
(async () => {
  const result = {formulas: payload.formulas.length, graphs: payload.graphs.length, errors: []};
  try {
    if (!window.katex || !window.mermaid) throw Error("Rendering assets unavailable");
    for (const f of payload.formulas) {
      try { katex.renderToString(f.tex, {displayMode: f.display, throwOnError: true, strict: "ignore"}); }
      catch (e) { result.errors.push({kind: "math", ...f, error: String(e)}); }
    }
    mermaid.initialize({startOnLoad: false, securityLevel: "strict"});
    for (const g of payload.graphs) {
      try { if (await mermaid.parse(g.code) === false) throw Error("Mermaid parse failed"); }
      catch (e) { result.errors.push({kind: "mermaid", ...g, error: String(e)}); }
    }
  } catch (e) { result.errors.push({kind: "runtime", error: String(e)}); }
  document.querySelector("#result").textContent = JSON.stringify(result);
})();</script>'''
    page = page.replace("KATEX", md2pdf.KATEX_JS).replace("MERMAID", md2pdf.MERMAID_JS).replace("PAYLOAD", payload)
    with tempfile.TemporaryDirectory(prefix="auto-rudder-doc-render-") as temp:
        audit = Path(temp) / "audit.html"
        audit.write_text(page, encoding="utf-8")
        cmd = ["google-chrome", "--headless=new", "--no-sandbox", "--disable-gpu",
               "--disable-dev-shm-usage", "--no-first-run", "--no-default-browser-check",
               f"--user-data-dir={temp}/profile", "--allow-file-access-from-files",
               "--virtual-time-budget=30000", "--dump-dom", audit.as_uri()]
        run = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
        if run.returncode:
            print(run.stderr[-3000:], file=sys.stderr)
            return 2
        match = re.search(r'<pre id="result">(.*?)</pre>', run.stdout, re.S)
        if not match or match[1] == "pending":
            print("Browser did not finish rendering checks", file=sys.stderr)
            return 2
        result = json.loads(html.unescape(match[1]))
    if args.report:
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return bool(result["errors"])


if __name__ == "__main__":
    sys.exit(main())
