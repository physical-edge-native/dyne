"""MkDocs hooks for the docs site (Issue #124).

The English user guides live in docs/en/ (single source). Before each build they are copied into
docs-site/guide/, with the tutorial images into docs-site/demo/tutorial/ (both git-ignored). Links in
the guides that point to other repository files (../../crates/..., ../design/...) are rewritten to
GitHub URLs, so `mkdocs build --strict` passes and the links work on the site.
"""
import os
import re
import shutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "docs", "en")
GUIDE = os.path.join(ROOT, "docs-site", "guide")
DEMO_SRC = os.path.join(ROOT, "docs", "demo", "tutorial")
DEMO = os.path.join(ROOT, "docs-site", "demo", "tutorial")
BLOB = "https://github.com/physical-edge-native/dyne/blob/main/"
TREE = "https://github.com/physical-edge-native/dyne/tree/main/"


def on_pre_build(config, **kwargs):
    for d in (GUIDE, DEMO):
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
    for name in os.listdir(SRC):
        if name.endswith(".md") and name != "README.md":
            shutil.copy(os.path.join(SRC, name), os.path.join(GUIDE, name))
    for name in os.listdir(DEMO_SRC):
        shutil.copy(os.path.join(DEMO_SRC, name), os.path.join(DEMO, name))


LINK = re.compile(r"\]\((\.\./[^)\s#]+)(#[^)]*)?\)")


def on_page_markdown(markdown, page, config, files, **kwargs):
    if not page.file.src_uri.startswith("guide/"):
        return markdown

    def fix(m):
        target, anchor = m.group(1), m.group(2) or ""
        if target.startswith("../demo/"):
            return m.group(0)  # copied next to the guides
        # resolve against docs/en/ in the repository
        repo_path = os.path.normpath(os.path.join("docs", "en", target)).replace(os.sep, "/")
        full = os.path.join(ROOT, repo_path)
        base = TREE if os.path.isdir(full) else BLOB
        return f"]({base}{repo_path}{anchor})"

    return LINK.sub(fix, markdown)
