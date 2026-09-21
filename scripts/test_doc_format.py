"""Regression tests for documentation formatting and HTML conversion."""
import html
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from md2pdf import build_md_parser, fix_relative_links, github_slug, protect_math, restore_math
from md2pdf import html_to_pdf
from scripts.format_docs import format_text, heading_ids, table_cells


class DocumentationTests(unittest.TestCase):
    def test_table_math_keeps_absolute_values(self):
        source = "| A | B |\n| --- | --- |\n| $|x| < 2$ | `|e| < 3` |\n"
        fixed = format_text(source)
        self.assertIn(r"$\vert x\vert  < 2$", fixed)
        self.assertIn(r"`\|e\| < 3`", fixed)
        self.assertEqual(len(table_cells(fixed.splitlines()[2])), 2)
        self.assertEqual(format_text(fixed), fixed)
        rendered = build_md_parser().render(fixed)
        self.assertIn("<code>|e| &lt; 3</code>", rendered)

    def test_heading_and_list_spacing(self):
        source = "# Heading\nText\n- A\n- B\n\n## Next\nBody\n"
        fixed = format_text(source)
        self.assertEqual(fixed, "# Heading\n\nText\n\n- A\n- B\n\n## Next\n\nBody\n")
        self.assertEqual(format_text(fixed), fixed)

    def test_existing_spacing_and_hard_breaks(self):
        source = "# A\n\nParagraph  \nsecond line\n\n- one\n- two\n\nEnd\n"
        self.assertEqual(format_text(source), source)

    def test_nested_list_keeps_structure(self):
        source = "- First\n  - Child\n  - Child 2\n- Second\n"
        self.assertEqual(format_text(source), source)
        nested = "- First\n\n    ~~~md\n    # Literal\n\n\n    $code$  \n    ~~~\n"
        self.assertEqual(format_text(nested), nested)

    def test_fenced_and_indented_examples_unchanged(self):
        source = "````md\n# Heading\n```\n$not math$\n\n\n````\n\n    $code$\n"
        self.assertEqual(format_text(source), source)
        safe, math = protect_math(source)
        self.assertEqual(safe, source)
        self.assertEqual(math, {})

    def test_tilde_and_multibacktick_code_are_not_math(self):
        source = "~~~text\n$x$\n~~~\n\n``$also ` code$`` and $a+1$.\n"
        safe, math = protect_math(source)
        self.assertEqual(list(math.values()), ["$a+1$"])
        self.assertIn("``$also ` code$``", safe)

    def test_math_round_trip_and_html_escaping(self):
        source = "$$\n\\begin{aligned}a&<b\\\\[4pt]c&>d\\end{aligned}\n$$\n"
        safe, math = protect_math(source)
        restored = restore_math(build_md_parser().render(safe), math)
        self.assertIn("&amp;&lt;", restored)
        self.assertIn(source.strip(), html.unescape(restored))
        self.assertNotIn("@@DOCMATH", restored)

    def test_parenthesized_math_and_escaped_currency(self):
        source = r"Pay \$10; \(x_1\) and \[x<y\]."
        _, math = protect_math(source)
        self.assertEqual(list(math.values()), [r"\(x_1\)", r"\[x<y\]"])

    def test_heading_ids_keep_underscores_and_spaces(self):
        self.assertEqual(github_slug("Script `tuner_debug_accept.py`"), "script-tuner_debug_acceptpy")
        self.assertEqual(github_slug("方差 / LQG"), "方差--lqg")
        self.assertEqual(github_slug("**Bold** [label](target.md)"), "bold-label")

    def test_duplicate_heading_ids_and_math_heading(self):
        source = "# A\n\n# A\n\n# A-1\n\n# $K_p$\n"
        safe, math = protect_math(source)
        rendered = build_md_parser().render(safe, {"math_placeholders": math})
        for name in heading_ids(source):
            self.assertIn(f'id="{name}"', rendered)
        self.assertEqual(heading_ids(source), {"a", "a-1", "a-1-1", "k_p"})

    def test_placeholder_collision_is_avoided(self):
        source = "@@DOCMATH0@@ and $x$."
        safe, math = protect_math(source)
        rendered = restore_math(build_md_parser().render(safe), math)
        self.assertIn("@@DOCMATH0@@", rendered)
        self.assertIn("$x$", rendered)

    def test_encoded_local_image_and_remote_image(self):
        source = '<img src="a%20b.png"><img src="https://example.com/a.png">'
        rendered = fix_relative_links(source, "/tmp/docs")
        self.assertIn('src="file:///tmp/docs/a%20b.png"', rendered)
        self.assertIn('src="https://example.com/a.png"', rendered)

    def test_failed_render_preserves_existing_pdf(self):
        with tempfile.TemporaryDirectory() as temp:
            target = Path(temp) / "document.pdf"
            target.write_bytes(b"previous PDF")
            def chrome(cmd, **kwargs):
                output = next(x.split("=", 1)[1] for x in cmd if x.startswith("--print-to-pdf="))
                Path(output).write_bytes(b"%PDF-incomplete")
                return subprocess.CompletedProcess(cmd, 0, '<html data-render-status="error">', '')
            with patch("md2pdf.subprocess.run", chrome):
                self.assertFalse(html_to_pdf("test", str(target)))
            self.assertEqual(target.read_bytes(), b"previous PDF")

    def test_completed_render_replaces_pdf(self):
        with tempfile.TemporaryDirectory() as temp:
            target = Path(temp) / "document.pdf"
            def chrome(cmd, **kwargs):
                output = next(x.split("=", 1)[1] for x in cmd if x.startswith("--print-to-pdf="))
                Path(output).write_bytes(b"%PDF-new")
                return subprocess.CompletedProcess(cmd, 0, '<html data-render-status="ready">', '')
            with patch("md2pdf.subprocess.run", chrome):
                self.assertTrue(html_to_pdf("test", str(target)))
            self.assertEqual(target.read_bytes(), b"%PDF-new")


if __name__ == "__main__":
    unittest.main()
