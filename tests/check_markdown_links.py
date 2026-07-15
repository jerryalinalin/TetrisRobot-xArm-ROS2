#!/usr/bin/env python3
"""Check local links in project-authored Markdown files."""

from __future__ import annotations

import re
from pathlib import Path
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
MARKDOWN_FILES = [ROOT / "README.md", ROOT / "THIRD_PARTY.md", *sorted((ROOT / "docs").rglob("*.md"))]
LINK_PATTERN = re.compile(r"!?\[[^\]]*\]\((?!https?://|#)([^)]+)\)")


def main() -> None:
    broken: list[str] = []
    for document in MARKDOWN_FILES:
        text = document.read_text(encoding="utf-8")
        for raw_target in LINK_PATTERN.findall(text):
            target = unquote(raw_target.split("#", 1)[0].strip())
            if target and not (document.parent / target).exists():
                broken.append(f"{document.relative_to(ROOT)} -> {target}")

    if broken:
        raise AssertionError("Broken local Markdown links:\n" + "\n".join(broken))
    print(f"PASS local Markdown links ({len(MARKDOWN_FILES)} files)")


if __name__ == "__main__":
    main()
