#!/usr/bin/env python3
"""Read-only source inventory; emit a self-contained, offline function atlas.

Run from anywhere: python3 docs/architecture/generate_call_atlas.py
No compilation, imports of project modules, network, or CodeGraph reindexing.
This is a conservative source scanner, NOT a C++/JS compiler or runtime tracer.
"""
from __future__ import annotations

import ast
from collections import Counter, defaultdict
import hashlib
import html
import json
from pathlib import Path
import re
import sqlite3
import subprocess

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SKIP = {".git", ".codegraph", "__pycache__", "node_modules", ".venv", "venv", "_deps"}
FUNCTIONS = []
SOURCES = {}
EDGES = {}
AUDIT = []
KEYWORDS = set("if for while switch catch sizeof alignof decltype static_cast dynamic_cast const_cast reinterpret_cast return else do try new delete function typeof void bool int double float char unsigned long assert".split())


def masked(src):
    """Keep offsets and newlines; remove comments and C++/JS string literals."""
    pattern = r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|`(?:\\.|[^`\\])*`'
    return re.sub(pattern, lambda m: re.sub(r"[^\n]", " ", m[0]), src)


def pairs(src):
    stack, result = [], {}
    for i, ch in enumerate(src):
        if ch in "({[":
            stack.append((ch, i))
        elif ch in ")}]" and stack and stack[-1][0] == dict(zip(")}]", "({["))[ch]:
            _, j = stack.pop()
            result[i] = j
            result[j] = i
    return result


def add(path, name, start, end, body, kind="function", scope="", node=None):
    src = SOURCES[path]
    line = src.count("\n", 0, start) + 1
    end_line = src.count("\n", 0, max(start, end - 1)) + 1
    uid = "f-" + hashlib.sha1(f"{path}:{start}:{name}".encode()).hexdigest()[:14]
    item = dict(id=uid, file=path, name=name, line=line, end_line=end_line,
                start=start, end=end, body=body, kind=kind, scope=scope,
                language={".py": "Python", ".sh": "Shell", ".html": "JavaScript"}.get(Path(path).suffix, "C++"),
                unresolved=[], _node=node)
    FUNCTIONS.append(item)
    return item


def enclosing(items, pos):
    hits = [f for f in items if f["start"] <= pos < f["end"]]
    return min(hits, key=lambda f: f["end"] - f["start"]) if hits else None


def scan_cpp(path):
    src = SOURCES[path]
    clean = masked(src)
    mate = pairs(clean)
    scopes = []
    for m in re.finditer(r"\b(namespace|class|struct)\s+([\w:]+)[^;{}]*\{", clean):
        opening = m.end() - 1
        if opening in mate:
            scopes.append((opening, mate[opening], m[2]))
    candidates = []
    pattern = r"(?<![\w])((?:[A-Za-z_]\w*::)*(?:~?[A-Za-z_]\w*|operator\s*\[\]))\s*\("
    for m in re.finditer(pattern, clean):
        name = re.sub(r"\s+", "", m[1])
        if name in KEYWORDS:
            continue
        opening = m.end() - 1
        closing = mate.get(opening)
        if closing is None:
            continue
        tail = re.match(r"\s*(?:(?:const|noexcept|override|final)\s*)*(?:->\s*[\w:<> &*]+\s*)?", clean[closing + 1:])
        j = closing + 1 + tail.end()
        default = re.match(r"=\s*default\s*;", clean[j:])
        if clean[j:j + 1] == ":" and clean[j:j + 2] != "::":
            # Constructor initializer list. Parenthesized/braced initializers
            # precede the actual body; jump over each initializer as a unit.
            k = j + 1
            while k < len(clean):
                init = re.match(r"\s*[\w:<>]+\s*([({])", clean[k:])
                if not init:
                    break
                op = k + init.end() - 1
                k = mate.get(op, op) + 1
                while k < len(clean) and clean[k].isspace():
                    k += 1
                if clean[k:k + 1] == ",":
                    k += 1
                else:
                    break
            j = k
        if default:
            end, body = j + default.end(), j
        elif clean[j:j + 1] == "{" and j in mate:
            end, body = mate[j] + 1, j + 1
        else:
            continue
        scope = "::".join(s[2] for s in sorted(scopes) if s[0] < m.start() < s[1])
        if name == "TEST":
            name = "TEST(" + re.sub(r"\s+", "", src[opening + 1:closing]) + ")"
            scope = ""
        full = f"{scope}::{name}" if scope else name
        candidates.append((m.start(), end, body, full, scope, bool(default)))
    # A constructor's final member initializer can look like a function;
    # the earliest candidate sharing its body is the declaration itself.
    by_body = {}
    for c in candidates:
        by_body.setdefault(c[2], c)
    for start, end, body, name, scope, default in by_body.values():
        add(path, name, start, end, body,
            "test" if name.startswith("TEST(") else "defaulted" if default else "function", scope)
    # Explicit lambdas, including callbacks passed to ParamStore.
    for m in re.finditer(r"\[[^\[\]\n]*\]\s*(?:\([^{};]*\))?\s*(?:mutable\s*)?(?:->\s*[\w:<> &*]+\s*)?\{", clean):
        op = m.end() - 1
        if op in mate:
            add(path, f"<lambda@{src.count(chr(10), 0, m.start()) + 1}>", m.start(), mate[op] + 1, op + 1, "lambda")


def scan_python(path):
    src = SOURCES[path]
    tree = ast.parse(src, filename=path)
    offsets = [0]
    for line in src.splitlines(keepends=True):
        offsets.append(offsets[-1] + len(line))

    def offset(node, end=False):
        lineno = node.end_lineno if end else node.lineno
        col = node.end_col_offset if end else node.col_offset
        # AST column offsets are UTF-8 bytes, not Unicode codepoints.
        line = src[offsets[lineno - 1]:offsets[lineno]]
        return offsets[lineno - 1] + len(line.encode()[:col].decode())

    def visit(node, scope=""):
        if isinstance(node, ast.ClassDef):
            for child in node.body:
                visit(child, f"{scope}.{node.name}".strip("."))
            return
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            local = node.name if hasattr(node, "name") else f"<lambda@{node.lineno}>"
            name = f"{scope}.{local}".strip(".")
            add(path, name, offset(node), offset(node, True), offset(node),
                "lambda" if isinstance(node, ast.Lambda) else "function", scope, node)
            scope = name
        for child in ast.iter_child_nodes(node):
            visit(child, scope)

    visit(tree)
    add(path, "<module entry>", 0, len(src), 0, "entry", node=tree)


def scan_js(path):
    src = SOURCES[path]
    # Mask HTML as well; offsets remain relative to the original HTML file.
    regions = [m.span(1) for m in re.finditer(r"<script[^>]*>([\s\S]*?)</script>", src)]
    buf = list(re.sub(r"[^\n]", " ", src))
    for a, b in regions:
        buf[a:b] = src[a:b]
    clean = masked("".join(buf))
    mate = pairs(clean)
    for m in re.finditer(r"\bfunction\s+([\w$]+)\s*\(", clean):
        close = mate[m.end() - 1]
        op = close + 1
        while clean[op].isspace():
            op += 1
        if clean[op] == "{" and op in mate:
            add(path, m[1], m.start(), mate[op] + 1, op + 1)
    for m in re.finditer(r"=>", clean):
        end_param = m.start() - 1
        while clean[end_param].isspace():
            end_param -= 1
        start = mate[end_param] if clean[end_param] == ")" else end_param
        while start and re.match(r"[\w$]", clean[start - 1]) and clean[end_param] != ")":
            start -= 1
        body = m.end()
        while clean[body].isspace():
            body += 1
        if clean[body] == "{":
            end = mate[body] + 1
            body += 1
        else:
            end = body
            while end < len(clean) and clean[end] not in ",;\n)}]":
                if clean[end] in "([{":
                    end = mate.get(end, end) + 1
                else:
                    end += 1
        prefix = src[max(0, start - 90):start]
        assignment = re.search(r"([^;{}\n]{1,70}?)\s*=\s*$", prefix)
        label = assignment[1].strip() if assignment else "callback"
        label = re.sub(r"^(?:const|let|var)\s+", "", label)
        add(path, f"{label} <arrow@{src.count(chr(10), 0, start) + 1}>", start, end, body, "lambda")
    for a, b in regions:
        add(path, "<script entry>", a, b, a, "entry")


def scan_shell(path):
    src = SOURCES[path]
    for m in re.finditer(r"(?m)^([A-Za-z_]\w*)\s*\(\s*\)\s*\{", src):
        # Current shell functions end with an unindented closing brace.
        end = re.search(r"(?m)^\}", src[m.end():])
        if end is None:
            raise ValueError(f"Unterminated shell function: {path}:{m[1]}")
        add(path, m[1], m.start(), m.end() + end.end(), m.end())
    add(path, "<shell entry>", 0, len(src), 0, "entry")


def edge(src, dst, line, evidence="lexical", kind="call"):
    key = (src["id"], dst["id"], kind)
    if key not in EDGES:
        EDGES[key] = dict(source=key[0], target=key[1], kind=kind, evidence=evidence, lines=[])
    if line not in EDGES[key]["lines"]:
        EDGES[key]["lines"].append(line)


def own_body(f, clean):
    body = list(clean[f["body"]:f["end"]])
    for nested in FUNCTIONS:
        if nested["file"] == f["file"] and nested["id"] != f["id"] and f["body"] <= nested["start"] < nested["end"] <= f["end"]:
            if nested["kind"] != "entry":
                a, b = nested["start"] - f["body"], nested["end"] - f["body"]
                body[a:b] = re.sub(r"[^\n]", " ", "".join(body[a:b]))
    return "".join(body)


def graph_cpp_js():
    cpp = [f for f in FUNCTIONS if f["language"] == "C++" and f["kind"] not in ("lambda", "test")]
    classes = {part for f in cpp for part in f["scope"].split("::") if part and part != "ar"}
    members = defaultdict(set)
    for path, src in SOURCES.items():
        if Path(path).suffix not in (".cpp", ".hpp"):
            continue
        clean = masked(src)
        for typ in classes:
            for m in re.finditer(r"\b" + re.escape(typ) + r"(?:\s*<[^;{}]+>)?\s*[*&]?\s+(\w+)\b", clean):
                members[(path, m[1])].add(typ)
    for f in FUNCTIONS:
        if f["language"] not in ("C++", "JavaScript") or f["kind"] == "defaulted":
            continue
        src = SOURCES[f["file"]]
        body = own_body(f, masked(src))
        for m in re.finditer(r"((?:[\w$]+::)*(?:[\w$]+|operator\s*\[\]))\s*\(", body):
            name = re.sub(r"\s+", "", m[1])
            if name in KEYWORDS:
                continue
            pos = f["body"] + m.start()
            line = src.count("\n", 0, pos) + 1
            receiver = re.search(r"([\w$]+)\s*(?:\.|->)\s*$", body[:m.start()])
            # Detect complex receivers too, leaving them unresolved for CodeGraph.
            member_call = re.search(r"(?:\.|->)\s*$", body[:m.start()]) is not None
            if f["language"] == "JavaScript":
                targets = [x for x in FUNCTIONS if x["file"] == f["file"] and x["name"] == name] if not member_call else []
            elif receiver:
                types = members[(f["file"], receiver[1])]
                targets = [x for x in cpp if x["name"].endswith("::" + name) and x["scope"].split("::")[-1] in types]
            elif member_call:
                targets = []
            elif "::" in name:
                targets = [x for x in cpp if x["name"] == name or x["name"].endswith("::" + name)]
            else:
                targets = [x for x in cpp if x["name"] == f["scope"] + "::" + name] if f["scope"] else []
                if not targets:
                    targets = [x for x in cpp if x["name"] in (name, "ar::" + name) and (x["file"] == f["file"] or x["name"].startswith("ar::"))]
                if not targets and name in classes:
                    targets = [x for x in cpp if x["name"].endswith("::" + name + "::" + name)]
            if len(targets) == 1:
                edge(f, targets[0], line)
            else:
                token = (receiver[1] + "." if receiver else "<expr>." if member_call else "") + name
                f["unresolved"].append(dict(name=token, line=line, reason="overload/ambiguous" if targets else "library/macro/indirect/unresolved"))
        # Lambdas are registered/defined here, not necessarily executed here.
        children = [x for x in FUNCTIONS if x["file"] == f["file"] and x["kind"] == "lambda" and f["body"] <= x["start"] < x["end"] <= f["end"]]
        for child in children:
            owner = enclosing([x for x in FUNCTIONS if x["file"] == f["file"] and x["id"] != child["id"]], child["start"])
            if owner and owner["id"] == f["id"]:
                edge(f, child, child["line"], "definition/registration, not execution", "callback")


def codegraph_cpp():
    db = ROOT / ".codegraph/codegraph.db"
    if not db.exists():
        AUDIT.append("CodeGraph DB unavailable; lexical C++/JS graph only.")
        return
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    con.row_factory = sqlite3.Row
    current = {p: hashlib.sha256(s.encode()).hexdigest() for p, s in SOURCES.items()}
    checked = {r["path"] for r in con.execute("SELECT path,content_hash FROM files") if current.get(r["path"]) == r["content_hash"]}
    nodes = {r["id"]: dict(r) for r in con.execute("SELECT * FROM nodes WHERE kind IN ('function','method') AND language='cpp'")}
    mapping, missing = {}, []
    for uid, n in nodes.items():
        candidates = [f for f in FUNCTIONS if f["file"] == n["file_path"] and f["line"] <= n["start_line"] <= f["end_line"] and f["kind"] != "entry"]
        exact = [f for f in candidates if f["name"] == n["qualified_name"]]
        if not exact and n["name"] == "TEST":
            exact = [f for f in candidates if f["kind"] == "test"]
        if exact:
            mapping[uid] = min(exact, key=lambda x: x["end"] - x["start"])
        elif n["file_path"] in checked:
            missing.append(f'{n["file_path"]}:{n["start_line"]} {n["qualified_name"]}')
    if missing:
        raise AssertionError("C++ inventory misses current CodeGraph definitions:\n" + "\n".join(missing))
    count, rejected = 0, 0
    for r in con.execute("SELECT * FROM edges WHERE kind='calls'"):
        a, b = nodes.get(r["source"]), nodes.get(r["target"])
        if not a or not b or a["file_path"] not in checked:
            continue
        x, y = mapping.get(a["id"]), mapping.get(b["id"])
        if x and y:
            meta = json.loads(r["metadata"] or "{}")
            # Index name-tail heuristics can confuse vector::size with our
            # RingBuffer::size, or an unrelated update/reset. Do not promote
            # those guesses merely because the source hash matches.
            if (x["id"], y["id"], "call") not in EDGES and meta.get("confidence", 0) < .85:
                rejected += 1
                continue
            line = r["line"] or x["line"]
            src = SOURCES[x["file"]]
            offset = sum(len(s) for s in src.splitlines(keepends=True)[:line - 1]) + (r["col"] or 0)
            owner = enclosing([f for f in FUNCTIONS if f["file"] == x["file"]], offset)
            if owner and owner["kind"] == "lambda":
                x = owner
            edge(x, y, line, "CodeGraph, source SHA-256 matched, confidence >=0.85 or lexically corroborated")
            count += 1
    AUDIT.append(f"CodeGraph: {len(checked)} source hashes matched; {count} call sites reused; {rejected} uncorroborated low-confidence call sites rejected; stale files scanned afresh; no reindex.")
    con.close()


def graph_python():
    for path in SOURCES:
        fs = [f for f in FUNCTIONS if f["file"] == path and f["language"] == "Python"]
        if not fs:
            continue
        by_name = {f["name"]: f for f in fs}
        classes = {f["scope"] for f in fs if f["name"].endswith(".__init__")}
        instances = defaultdict(set)
        tree = next(f["_node"] for f in fs if f["kind"] == "entry")
        for node in ast.walk(tree):
            if isinstance(node, (ast.Assign, ast.AnnAssign)) and isinstance(node.value, ast.Call):
                typ = ast.unparse(node.value.func)
                if typ in classes:
                    for target in node.targets if isinstance(node, ast.Assign) else [node.target]:
                        instances[ast.unparse(target)].add(typ)

        def resolve(name, owner):
            candidates = []
            if name in by_name:
                candidates.append(by_name[name])
            parts = owner["name"].split(".")
            for n in range(len(parts), 0, -1):
                scoped = ".".join(parts[:n]) + "." + name
                if scoped in by_name:
                    candidates.append(by_name[scoped])
            if name.startswith("self.") and owner["scope"]:
                scoped = owner["scope"].split(".")[0] + "." + name[5:]
                if scoped in by_name:
                    candidates.append(by_name[scoped])
            if "." in name:
                receiver, method = name.rsplit(".", 1)
                for typ in instances[receiver]:
                    if typ + "." + method in by_name:
                        candidates.append(by_name[typ + "." + method])
            if name in classes:
                candidates.append(by_name[name + ".__init__"])
            unique = {f["id"]: f for f in candidates}
            return next(iter(unique.values())) if len(unique) == 1 else None

        for f in fs:
            def walk(node, root=False):
                if not root and isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
                    return
                if isinstance(node, ast.Call):
                    name = ast.unparse(node.func)
                    dst = resolve(name, f)
                    if dst:
                        edge(f, dst, node.lineno, "Python AST + local name resolution")
                    else:
                        f["unresolved"].append(dict(name=name, line=node.lineno, reason="library/dynamic/unresolved"))
                    for arg in list(node.args) + [kw.value for kw in node.keywords]:
                        if isinstance(arg, (ast.Name, ast.Attribute)):
                            target = resolve(ast.unparse(arg), f)
                            if target:
                                edge(f, target, node.lineno, "callable argument; invocation controlled by receiver", "callback")
                for child in ast.iter_child_nodes(node):
                    walk(child)
            walk(f["_node"], True)
            for child in fs:
                if child["kind"] != "lambda" or child["id"] == f["id"]:
                    continue
                owner = enclosing([x for x in fs if x["id"] != child["id"]], child["start"])
                if owner and owner["id"] == f["id"]:
                    edge(f, child, child["line"], "lambda defined here; not necessarily invoked", "callback")


def graph_shell():
    for f in FUNCTIONS:
        if f["language"] != "Shell":
            continue
        body = own_body(f, SOURCES[f["file"]])
        for target in FUNCTIONS:
            if target["file"] == f["file"] and target["kind"] != "entry":
                for m in re.finditer(r"(?<![\w])" + re.escape(target["name"]) + r"\s", body):
                    edge(f, target, SOURCES[f["file"]].count("\n", 0, f["body"] + m.start()) + 1, "shell token")
        for token in ("$SIM", "$ANIM", "python3", "ffmpeg"):
            if token in body:
                f["unresolved"].append(dict(name=token, line=f["line"], reason="external process, not same-process call"))


def semantic_edges():
    # Few runtime bindings whose identities are explicit in current source.
    # They remain dashed; the atlas never claims a runtime trace.
    def one(path, name):
        result = [f for f in FUNCTIONS if f["file"] == path and f["name"] == name]
        return result[0] if len(result) == 1 else None
    ui = "control/ui/debug_ui.html"
    debug = "control/include/tuner_debug_api.hpp"
    tuner = "control/include/auto_tuner.hpp"
    for source_name, expression, target_name in (
        ("oscFeed", "t_.oscDetector_.update", "ar::AutoTuner::OscillationDetector::update"),
        ("oscReset", "t_.oscDetector_.reset", "ar::AutoTuner::OscillationDetector::reset"),
        ("oscAmplitude", "t_.oscDetector_.amplitude", "ar::AutoTuner::OscillationDetector::amplitude"),
        ("snapshot", "t_.oscDetector_.amplitude", "ar::AutoTuner::OscillationDetector::amplitude"),
        ("snapshot", "t_.windowErr_.size", "ar::RingBuffer::size"),
    ):
        source = one(debug, "ar::TunerDebugApi::" + source_name)
        target = one(tuner, target_name)
        if source and target:
            segment = SOURCES[debug][source["body"]:source["end"]]
            if expression in segment:
                pos = source["body"] + segment.index(expression)
                edge(source, target, SOURCES[debug].count("\n", 0, pos) + 1,
                     "manually checked member type + current expression guard")
    for f in FUNCTIONS:
        if f["file"] == ui:
            src = SOURCES[ui][f["body"]:f["end"]]
            if "setInterval(controlStep" in src and f["kind"] != "entry":
                target = one(ui, "controlStep")
                if target:
                    edge(f, target, f["line"], "setInterval(controlStep, ...) timer registration", "callback")


def svg(f, outgoing, by_id):
    """One SVG per function, all outgoing own-code edges, with linked nodes."""
    rows = max(1, len(outgoing))
    height = 52 + rows * 50
    e = html.escape
    center = height / 2
    marker_id = f["id"] + "-arrow"
    fit = ' textLength="375" lengthAdjust="spacingAndGlyphs"' if len(f["name"]) > 47 else ""
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1120 {height}" role="img" aria-label="{e(f["name"])} 调用关系">',
             f'<defs><marker id="{marker_id}" markerWidth="7" markerHeight="7" refX="6" refY="3" orient="auto"><path d="M0 0 L6 3 L0 6" fill="none" stroke="#547893"/></marker></defs>',
             f'<rect x="8" y="{center - 28}" width="410" height="56" rx="9" fill="#17334d"/>',
             f'<text x="22" y="{center - 4}" fill="white" font-size="13"{fit}>{e(f["name"])}</text>',
             f'<text x="22" y="{center + 17}" fill="#a9d6ed" font-size="12">{e(f["kind"])} · L{f["line"]}–{f["end_line"]}</text>']
    if not outgoing:
        parts += [f'<path d="M418 {center} H535" stroke="#547893" marker-end="url(#{marker_id})"/>',
                  f'<text x="550" y="{center + 5}" font-size="15" fill="#4a6170">无已解析的项目内调用；库调用及未解析点见下表</text>']
    for i, ed in enumerate(outgoing):
        target = by_id[ed["target"]]
        y = 30 + i * 50
        callback = ed["kind"] == "callback"
        dash = 'stroke-dasharray="5 4"' if callback else ""
        parts += [f'<path d="M418 {center} C475 {center},475 {y + 19},538 {y + 19}" fill="none" stroke="#547893" {dash} marker-end="url(#{marker_id})"/>',
                  f'<a href="#{target["id"]}"><rect x="545" y="{y}" width="565" height="39" rx="6" fill="{"#fff3d6" if callback else "#e6f2f6"}" stroke="#aec9d6"/>',
                  f'<text x="557" y="{y + 16}" font-size="12" fill="#17334d">{e(target["name"][:79])}</text>',
                  f'<text x="557" y="{y + 31}" font-size="10" fill="#456">{"注册/传递回调" if callback else "静态调用"} · 调用点 L{", ".join(map(str, sorted(ed["lines"])))}</text></a>']
    return "".join(parts) + "</svg>"


def export():
    FUNCTIONS.sort(key=lambda f: (f["file"], f["line"], f["start"], f["name"]))
    by_id = {f["id"]: f for f in FUNCTIONS}
    outgoing, incoming = defaultdict(list), defaultdict(list)
    for ed in EDGES.values():
        outgoing[ed["source"]].append(ed)
        incoming[ed["target"]].append(ed)
    public = [{k: v for k, v in f.items() if not k.startswith("_")} for f in FUNCTIONS]
    hashes = {p: hashlib.sha256(s.encode()).hexdigest() for p, s in SOURCES.items()}
    stats = dict(source_files=len(SOURCES), definitions=sum(f["kind"] != "entry" for f in FUNCTIONS),
                 entries=sum(f["kind"] == "entry" for f in FUNCTIONS), cards=len(FUNCTIONS),
                 edges=len(EDGES), by_language=dict(Counter(f["language"] for f in FUNCTIONS if f["kind"] != "entry")),
                 by_kind=dict(Counter(f["kind"] for f in FUNCTIONS)),
                 test_cases=sum(f["kind"] == "test" for f in FUNCTIONS))
    payload = dict(scope="Repository-owned .cpp/.hpp/.py/.sh plus control/ui/debug_ui.html; excludes build/cache/vendor and docs/architecture generator/output itself.",
                   method="Fresh lexical C++/JS/Shell definition scan, Python AST; hash-verified read-only CodeGraph C++ call edges; conservative local resolution. Not compiler-complete and not execution traces.",
                   stats=stats, audit=AUDIT, source_sha256=hashes, functions=public, edges=list(EDGES.values()))
    (OUT / "functions.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    counts = defaultdict(Counter)
    for f in FUNCTIONS:
        counts[f["file"]]["entry" if f["kind"] == "entry" else "definition"] += 1
    index = ["# 全函数索引", "", "自动生成；阅读方法、边界与流程说明见 [架构主文档](../项目架构与全函数调用说明.md)。", "",
             f"覆盖 **{stats['source_files']} 个源码文件、{stats['definitions']} 个显式函数/方法/测试/匿名函数**，另有 {stats['entries']} 个脚本入口；共 {stats['cards']} 张函数关系图。", "",
             "打开 [离线交互图谱](call_atlas.html) 可按文件、函数名筛选；实线是静态调用关系，虚线是回调定义/注册/传递，不表示立即执行。", "",
             "每个节点均有源文件、当前行号、调用者、全部已解析的项目内被调用者及未解析调用点。没有连线不等于函数永远不会被执行。", "",
             "| 文件 | 显式定义 | 脚本入口 |", "|---|---:|---:|"]
    for path in SOURCES:
        index.append(f"| [{path}](../../{path}) | {counts[path]['definition']} | {counts[path]['entry']} |")
    for path in SOURCES:
        index += ["", f"## {path}", "", "| 函数/回调 | 当前源码行 | 图谱 |", "|---|---:|---|"]
        for f in FUNCTIONS:
            if f["file"] == path:
                index.append(f"| `{f['name'].replace('|', '&#124;')}` | {f['line']}–{f['end_line']} | [调用图](call_atlas.html#{f['id']}) |")
    (OUT / "全函数索引.md").write_text("\n".join(index) + "\n", encoding="utf-8")
    e = html.escape
    cards = []
    for f in FUNCTIONS:
        callers = "、".join(f'<a href="#{ed["source"]}">{e(by_id[ed["source"]]["name"])}</a>' for ed in incoming[f["id"]]) or "未发现已解析的项目内调用者（可能为外部入口、注册回调或未使用辅助函数）"
        unresolved = "".join(f'<tr><td><code>{e(x["name"])}</code></td><td>{x["line"]}</td><td>{e(x["reason"])}</td></tr>' for x in f["unresolved"])
        evidence = "".join(f'<li><a href="#{ed["target"]}">{e(by_id[ed["target"]]["name"])}</a>：{e(ed["evidence"])}</li>' for ed in outgoing[f["id"]])
        # Source link is relative, but line is also printed (file:// line anchors
        # are not universally supported). Embedded excerpt works fully offline.
        lines = SOURCES[f["file"]].splitlines()
        excerpt = "\n".join(f"{i + 1:4d}  {lines[i]}" for i in range(f["line"] - 1, f["end_line"])) if f["kind"] != "entry" else "脚本入口代表模块顶层；命名函数体单独计入各自节点。请点击上方源文件查看完整模块。"
        cards.append(f'''<article id="{f['id']}" data-search="{e((f['file']+' '+f['name']+' '+f['language']).lower(), quote=True)}">
<h2>{e(f['name'])}</h2><p class="meta"><a href="../../{e(f['file'])}">{e(f['file'])}</a> · L{f['line']}–{f['end_line']} · {f['language']} / {f['kind']}</p>
{svg(f, outgoing[f['id']], by_id)}<p><b>调用者/注册位置：</b>{callers}</p>
<details><summary>边的证据（{len(outgoing[f['id']])} 条）</summary><ul>{evidence or '<li>无已解析项目内边</li>'}</ul></details>
<details><summary>库、宏、重载或未解析调用点（{len(f['unresolved'])} 处）</summary><p>这些项不是编译错误；其中可能包括同一调用点的 CodeGraph 补充解析。参照上方证据。</p><table><tr><th>表达式</th><th>行</th><th>分类</th></tr>{unresolved}</table></details>
<details><summary>当前源码摘录</summary><pre>{e(excerpt)}</pre></details></article>''')
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>自动舵 · 全函数调用图谱</title>
<style>:root{color-scheme:light}*{box-sizing:border-box}body{margin:0;background:#eef3f7;color:#193348;font:15px/1.65 system-ui,"Noto Sans CJK JP",sans-serif}header{position:sticky;top:0;z-index:2;background:#102d46;color:white;padding:14px max(24px,calc((100vw - 1200px)/2));box-shadow:0 3px 16px #1233}h1{font-size:23px;margin:0 0 8px}header a{color:#a3e0fa}input{width:min(700px,80vw);padding:11px;border:0;border-radius:7px;font:inherit}main{max-width:1200px;margin:auto;padding:22px}article{background:white;border:1px solid #d5e1e9;border-radius:12px;padding:22px;margin:22px 0;scroll-margin-top:170px}article:target{outline:3px solid #e4a843}h2{font-size:19px;overflow-wrap:anywhere;margin:0 0 4px}.meta{color:#577080;margin-top:0}svg{width:100%;height:auto}a{color:#006a9b}details{margin:9px 0}summary{cursor:pointer}pre{background:#f3f6f8;overflow:auto;padding:14px;font:12px/1.65 monospace}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccd8e2;padding:6px;text-align:left}code{overflow-wrap:anywhere}#count{margin-left:12px}.note{padding:16px;border-left:5px solid #e4a843;background:#fff8e7}button{cursor:pointer;border:1px solid #bfd4df;background:white;padding:8px;border-radius:5px}</style>
<header><h1>自动舵 · 全函数调用图谱</h1><input id="q" placeholder="搜索函数、文件或语言，例如 AutoTuner / test_ / JavaScript"><span id="count"></span><div><a href="项目架构说明.html">架构主文档</a> · <a href="全函数索引.md">全函数索引</a> · <a href="functions.json">机器可读数据</a></div></header><main>
<p class="note">这是静态关系图，不是运行轨迹或完整控制流图。实线表示源码调用依赖（可能在分支、循环中发生）；虚线表示匿名函数定义/注册/传递，执行时机由运行库、事件或外部回调决定。调用顺序和条件请看主文档。第三方实现、隐式构造/析构、模板实例化及动态目标不作为“已完整解析”处理。</p>
<p>SUMMARY</p>CARDS</main><script>
const q=document.getElementById('q'), cards=[...document.querySelectorAll('article')], count=document.getElementById('count');
function filter(){let n=0;const words=q.value.toLowerCase().trim().split(/\\s+/).filter(Boolean);for(const a of cards){const ok=words.every(w=>a.dataset.search.includes(w));a.hidden=!ok;if(ok)n++;}count.textContent=n+' / '+cards.length;}
q.addEventListener('input',filter);
function reveal(){const target=document.getElementById(location.hash.slice(1));if(target?.tagName==='ARTICLE'){q.value='';filter();requestAnimationFrame(()=>target.scrollIntoView());}}
window.addEventListener('hashchange',reveal);filter();reveal();
</script></html>'''
    page = page.replace("SUMMARY", f"{stats['source_files']} 个源码文件 · {stats['definitions']} 个显式定义 · {stats['entries']} 个脚本入口 · {stats['edges']} 条已解析静态关系。每个定义对应一张图。")
    page = page.replace("CARDS", "\n".join(cards))
    (OUT / "call_atlas.html").write_text(page, encoding="utf-8")
    assert page.count("<article id=") == len(FUNCTIONS)
    assert len(by_id) == len(FUNCTIONS)
    assert all(ed["source"] in by_id and ed["target"] in by_id for ed in EDGES.values())
    # Every fresh Python def/lambda and every TEST macro must appear once.
    for path, src in SOURCES.items():
        fs = [f for f in FUNCTIONS if f["file"] == path]
        if path.endswith(".py"):
            expected = sum(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)) for n in ast.walk(ast.parse(src)))
            assert sum(f["kind"] != "entry" for f in fs) == expected, path
        if path.endswith(".cpp"):
            assert sum(f["kind"] == "test" for f in fs) == len(re.findall(r"\bTEST\s*\(", masked(src))), path
    print(json.dumps(stats, ensure_ascii=False, indent=2))
    print("\n".join(AUDIT))
    print("PASS: all inventory IDs, SVG cards, edge targets, Python definitions and TEST macros checked.")


def main():
    paths = subprocess.check_output(["rg", "--files", "--hidden", "-g", "*.cpp", "-g", "*.hpp", "-g", "*.py", "-g", "*.sh", "-g", "!docs/architecture/**"], cwd=ROOT, text=True).splitlines()
    paths.append("control/ui/debug_ui.html")
    for path in sorted(set(paths)):
        if any(part in SKIP or part.startswith("build") for part in Path(path).parts):
            continue
        if not (path.startswith("control/") or path.startswith("docs/") or path == "md2pdf.py"):
            continue
        SOURCES[path] = (ROOT / path).read_text(encoding="utf-8")
    for path in SOURCES:
        {".py": scan_python, ".cpp": scan_cpp, ".hpp": scan_cpp, ".html": scan_js, ".sh": scan_shell}[Path(path).suffix](path)
    graph_cpp_js()
    codegraph_cpp()
    graph_python()
    graph_shell()
    semantic_edges()
    export()


if __name__ == "__main__":
    main()
