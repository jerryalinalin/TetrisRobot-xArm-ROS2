#!/usr/bin/env python3
"""Offline regression checks that do not require ROS2 or robot hardware."""

from __future__ import annotations

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def test_calibration_imports() -> None:
    path = ROOT / "scripts/calibration/calibrate_pick.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports_os = any(
        isinstance(node, ast.Import) and any(alias.name == "os" for alias in node.names)
        or isinstance(node, ast.ImportFrom) and node.module == "os"
        for node in ast.walk(tree)
    )
    uses_os = any(
        isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "os"
        for node in ast.walk(tree)
    )
    require(not uses_os or imports_os, "calibrate_pick.py uses os without importing it")


def test_strategy_state_and_bounds() -> None:
    path = ROOT / "src/block_sorter/include/block_sorter/strategy.h"
    source = path.read_text(encoding="utf-8")
    strategy_body = source[source.index("void strategy()") :]

    require(
        re.search(r"\bflag\s*=\s*(?:false|0)\s*;", strategy_body) is not None,
        "strategy() must reset flag before every search",
    )
    require(
        re.search(r"\bID\s*=\s*0\s*;", strategy_body) is not None,
        "strategy() must reset ID before every search",
    )
    require(
        re.search(r"if\s*\(x\s*>=\s*14\)\s*return\s*;", source) is not None,
        "dfs() must stop before reading beyond row 13",
    )
    require(
        re.search(r"if\s*\(x\s*==\s*13\)[\s\S]{0,500}\bflag\s*=\s*1", source)
        is None,
        "dfs() must not treat reaching the final row as success",
    )


def test_strategy_data_shape() -> None:
    data = (ROOT / "src/block_sorter/data/t.txt").read_text(encoding="utf-8")
    rotations = [int(value) for value in re.findall(r"^====(\d+)\s*$", data, re.MULTILINE)]
    require(len(rotations) == 7, "strategy data must define seven block types")
    require(sum(rotations) == 19, "strategy data must define nineteen rotations")


def test_publish_layout_and_privacy() -> None:
    root_scripts = ["start_color.sh", "start_pipeline.sh", "calibrate_pick.py", "calibrate_place.py"]
    require(
        all(not (ROOT / name).exists() for name in root_scripts),
        "runtime and calibration scripts must not remain in the repository root",
    )
    required = [
        "scripts/start_color.sh",
        "scripts/start_pipeline.sh",
        "scripts/calibration/calibrate_pick.py",
        "scripts/calibration/calibrate_place.py",
        "config/workcell.example.yaml",
    ]
    require(all((ROOT / path).exists() for path in required), "organized publish files are missing")

    private_tokens = [
        "W-" + "Dych",
        "Jerry" + " Lin",
        "jerrylin" + "@todo.todo",
        "192.168.1." + "205",
        "C:" + "\\Users\\" + "aijer",
    ]
    private_pattern = re.compile("|".join(re.escape(token) for token in private_tokens))
    authored_files = [
        ROOT / "README.md",
        ROOT / "LICENSE",
        ROOT / "docs/architecture.md",
        ROOT / "docs/technical_report_summary.md",
        ROOT / "scripts/start_color.sh",
        ROOT / "scripts/start_pipeline.sh",
        ROOT / "src/block_sorter/package.xml",
        ROOT / "src/color_detector/package.xml",
    ]
    for path in authored_files:
        require(private_pattern.search(path.read_text(encoding="utf-8")) is None, f"private data in {path}")


def main() -> None:
    tests = [
        test_calibration_imports,
        test_strategy_state_and_bounds,
        test_strategy_data_shape,
        test_publish_layout_and_privacy,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    main()
