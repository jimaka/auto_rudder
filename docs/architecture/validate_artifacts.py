#!/usr/bin/env python3
"""Validate generated docs only; do not build or execute control algorithms."""
from collections import Counter
from hashlib import sha256
from html.parser import HTMLParser
import json
from pathlib import Path
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

from markdown_it import MarkdownIt
from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


class Page(HTMLParser):
    def __init__(self, text):
        super().__init__()
        self.ids, self.links, self.cards, self.svgs = [], [], [], 0
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            self.ids.append(attrs["id"])
        if tag == "article":
            self.cards.append(attrs.get("id"))
        if tag == "svg":
            self.svgs += 1
        for name in ("href", "src"):
            if attrs.get(name):
                self.links.append(attrs[name])


def main():
    data = json.loads((HERE / "functions.json").read_text())
    by_id = {f["id"]: f for f in data["functions"]}
    assert len(by_id) == data["stats"]["cards"]
    for filename, digest in data["source_sha256"].items():
        assert sha256((ROOT / filename).read_bytes()).hexdigest() == digest, filename
    pages = {}
    for filename in ("call_atlas.html", "项目架构说明.html"):
        path = HERE / filename
        page = Page(path.read_text())
        assert not [key for key, n in Counter(page.ids).items() if n != 1], filename
        pages[path.resolve()] = page
    atlas = pages[(HERE / "call_atlas.html").resolve()]
    assert set(atlas.cards) == set(by_id)
    assert atlas.svgs == len(by_id)
    for edge in data["edges"]:
        assert edge["source"] in by_id and edge["target"] in by_id

    checked = 0
    def link(path, value):
        nonlocal checked
        parsed = urlsplit(value)
        if parsed.scheme:
            return
        target = (path.parent / unquote(parsed.path)).resolve() if parsed.path else path.resolve()
        assert target.exists(), (str(path), value)
        if parsed.fragment and target in pages:
            assert unquote(parsed.fragment) in pages[target].ids, (str(path), value)
        checked += 1

    for path, page in pages.items():
        for value in page.links:
            link(path, value)
    parser = MarkdownIt("commonmark").enable("table")
    for path in (HERE.parent / "项目架构与全函数调用说明.md", HERE / "全函数索引.md", HERE / "figures.md"):
        def walk(tokens):
            for token in tokens:
                for name in ("href", "src"):
                    if value := token.attrGet(name):
                        link(path, value)
                if token.children:
                    walk(token.children)
        walk(parser.parse(path.read_text()))
    pngs = sorted((HERE / "figures").glob("*.png"))
    svgs = sorted((HERE / "figures").glob("*.svg"))
    assert len(pngs) == len(svgs) == 14
    for path in pngs:
        with Image.open(path) as img:
            img.verify()
    for path in svgs:
        ET.parse(path)

    # Guard against the actual false-positive name-tail matches encountered
    # in this index, and the most important architecture boundary mistakes.
    targets = {}
    for f in by_id.values():
        targets[(f["file"], f["name"])] = {
            by_id[e["target"]]["name"] for e in data["edges"] if e["source"] == f["id"]}
    load = targets[("control/include/param_store.hpp", "ar::ParamStore::load")]
    assert "ar::RingBuffer::size" not in load
    assert "ar::ControlParams::validateRange" not in load
    for method, expected in (("oscFeed", "update"), ("oscReset", "reset")):
        own = targets[("control/include/tuner_debug_api.hpp", "ar::TunerDebugApi::" + method)]
        assert own == {"ar::AutoTuner::OscillationDetector::" + expected}, own
    for f in by_id.values():
        if f["language"] == "JavaScript":
            assert all(by_id[e["target"]]["language"] == "JavaScript" for e in data["edges"] if e["source"] == f["id"])
    print(f"PASS: {len(data['source_sha256'])} current source hashes; {len(by_id)} unique function/entry SVG cards; {len(data['edges'])} edge targets.")
    print(f"PASS: {checked} local links/fragments; 14 PNG + 14 SVG; both HTML pages have unique IDs.")
    print("PASS: storage/debug/UI boundary regression checks. Control algorithms were not executed.")


if __name__ == "__main__":
    main()
