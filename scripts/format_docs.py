#!/usr/bin/env python3
"""Conservative Markdown formatting and local-link checks (no network access)."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from md2pdf import build_md_parser, github_slug, protect_math

INLINE = re.compile(r"(?P<code>`+)(?P<body>.*?)(?P=code)(?!`)|(?<![\\$])\$(?!\$)(?P<math>[^$\n]+?)(?<!\\)\$(?!\$)")
SEPARATOR = re.compile(r"^\s*\|?\s*:?-{3,}:?\s*(?:\|\s*:?-{3,}:?\s*)+\|?\s*$")


def files():
    result = subprocess.run(
        ["rg", "--files", "--hidden", "-g", "*.md", "-g", "!.git/**",
         "-g", "!.codegraph/**", "-g", "!**/node_modules/**", "-g", "!build*/**"],
        cwd=ROOT, capture_output=True, text=True, check=True)
    return sorted(ROOT / name for name in result.stdout.splitlines())


def protected_lines(lines):
    """Keep fenced examples, indented code and display equations unchanged."""
    protected = set()
    for token in build_md_parser().parse("\n".join(lines)):
        if token.type in ("fence", "code_block") and token.map:
            protected.update(range(*token.map))
    math_end = None
    for i, line in enumerate(lines):
        if i in protected:
            continue
        stripped = line.strip()
        if math_end:
            protected.add(i)
            if stripped.endswith(math_end):
                math_end = None
            continue
        if stripped.startswith("$$"):
            protected.add(i)
            if not (len(stripped) > 2 and stripped.endswith("$$")):
                math_end = "$$"
        elif stripped.startswith(r"\["):
            protected.add(i)
            if not stripped.endswith(r"\]"):
                math_end = r"\]"
    return protected


def table_ranges(lines, protected):
    for i, line in enumerate(lines):
        if i and i not in protected and SEPARATOR.fullmatch(line) and "|" in lines[i - 1]:
            end = i + 1
            while end < len(lines) and lines[end].strip() and "|" in lines[end] and end not in protected:
                end += 1
            yield i - 1, end


def table_cells(line):
    # A pipe with an odd number of preceding backslashes is not a separator.
    cells, start = [], 0
    for i, char in enumerate(line):
        if char != "|":
            continue
        backslashes, j = 0, i - 1
        while j >= 0 and line[j] == "\\":
            backslashes += 1
            j -= 1
        if backslashes % 2 == 0:
            cells.append(line[start:i].strip())
            start = i + 1
    cells.append(line[start:].strip())
    if cells and not cells[0]:
        cells.pop(0)
    if cells and not cells[-1]:
        cells.pop()
    return cells


def format_text(text):
    lines = text.splitlines()
    protected = protected_lines(lines)
    for start, end in table_ranges(lines, protected):
        for i in range(start, end):
            def escape_pipe(match):
                # \vert preserves absolute-value notation; \| in TeX is a norm.
                replacement = r"\vert " if match["math"] is not None else r"\|"
                return re.sub(r"(?<!\\)\|", lambda _: replacement, match[0])
            lines[i] = INLINE.sub(escape_pipe, lines[i])

    # Use parser block boundaries so nested lists and code are not split apart.
    blank_before, blank_after = set(), set()
    for token in build_md_parser().parse("\n".join(lines)):
        if not token.map:
            continue
        start, end = token.map
        if start in protected and token.type != "fence":
            continue
        if token.type == "heading_open":
            blank_before.add(start)
            blank_after.add(end - 1)
        elif token.level == 0 and token.type in ("bullet_list_open", "ordered_list_open", "table_open", "fence"):
            blank_before.add(start)
            blank_after.add(end - 1)

    result = []
    for i, line in enumerate(lines):
        if i in blank_before and result and result[-1].strip():
            result.append("")
        if i not in protected:
            suffix = "  " if line.endswith("  ") and line.strip() else ""
            line = line.rstrip() + suffix
            if not line and result and not result[-1]:
                continue
        result.append(line)
        if i in blank_after and line.strip() and i + 1 < len(lines) and lines[i + 1].strip():
            result.append("")
    return "\n".join(result).rstrip("\n") + "\n"


def walk(tokens):
    for token in tokens:
        yield token
        if token.children:
            yield from walk(token.children)


def heading_ids(text):
    used, result = set(), set()
    tokens = build_md_parser().parse(text)
    for i, token in enumerate(tokens):
        if token.type == "heading_open":
            base = github_slug(tokens[i + 1].content)
            slug, count = base, 0
            while slug in used:
                count += 1
                slug = f"{base}-{count}"
            used.add(slug)
            result.add(slug)
    # Explicit anchors are also valid destinations.
    result.update(re.findall(r'<(?:a|h[1-6])\b[^>]*\b(?:id|name)=[\"\']([^\"\']+)', text))
    return result


def check_documents(paths):
    issues, counts = [], Counter()
    ids = {path.resolve(): heading_ids(path.read_text(encoding="utf-8")) for path in paths}
    for path in paths:
        text = path.read_text(encoding="utf-8")
        name = str(path.relative_to(ROOT))
        lines = text.splitlines()
        if format_text(text) != text:
            issues.append(f"{name}: format differs (run --fix)")
        for start, end in table_ranges(lines, protected_lines(lines)):
            width = len(table_cells(lines[start + 1]))
            counts["tables"] += 1
            for i in range(start, end):
                actual = len(table_cells(lines[i]))
                if actual != width:
                    issues.append(f"{name}:{i + 1}: table has {actual} cells, expected {width}")
        safe, _ = protect_math(text)
        for token in walk(build_md_parser().parse(safe)):
            for attr in ("src", "href"):
                value = token.attrGet(attr)
                if not value:
                    continue
                parsed = urlsplit(value)
                if parsed.scheme or value.startswith("//"):
                    continue  # Do not claim to check remote URLs.
                target = (path.parent / unquote(parsed.path)).resolve() if parsed.path else path.resolve()
                counts["local_links"] += 1
                if token.type == "image":
                    counts["images"] += 1
                if not target.exists():
                    issues.append(f"{name}: missing local target: {unquote(value)}")
                elif parsed.fragment and target in ids and unquote(parsed.fragment) not in ids[target]:
                    issues.append(f"{name}: missing heading: {unquote(value)}")
        for token in build_md_parser().parse(text):
            if token.type == "fence":
                counts["code_blocks"] += 1
                last = lines[token.map[1] - 1].strip()
                if not re.fullmatch(re.escape(token.markup[0]) + "{" + str(len(token.markup)) + r",}", last):
                    issues.append(f"{name}:{token.map[0] + 1}: unclosed fence")
    return issues, counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fix", action="store_true", help="Apply safe whitespace and table-pipe repairs")
    parser.add_argument("--check", action="store_true", help="Check only (the default)")
    parser.add_argument("--report", type=Path, help="Write a JSON audit report")
    args = parser.parse_args()
    paths, changed = files(), []
    if args.fix:
        for path in paths:
            original = path.read_text(encoding="utf-8")
            fixed = format_text(original)
            if fixed != original:
                path.write_text(fixed, encoding="utf-8")
                changed.append(str(path.relative_to(ROOT)))
    issues, counts = check_documents(paths)
    result = {"documents": len(paths), "changed": changed, **dict(counts), "issues": issues}
    if args.report:
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return bool(issues)


if __name__ == "__main__":
    sys.exit(main())
