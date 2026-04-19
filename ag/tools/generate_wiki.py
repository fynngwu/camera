"""Generate a Markdown index of modules, classes, and functions."""
from __future__ import annotations

import ast
from pathlib import Path
from typing import List, Tuple

ROOT = Path(__file__).resolve().parents[1]
SRC_DIRS = [
    ROOT / "common",
    ROOT / "ground_robot",
    ROOT / "ground_station",
]
OUT_PATH = ROOT / "wiki" / "function_index.md"


def iter_python_files() -> List[Path]:
    """Return project Python files that should appear in the wiki index."""
    files: List[Path] = []
    for src in SRC_DIRS:
        files.extend(sorted(src.rglob("*.py")))
    return files


def summarize_doc(node: ast.AST) -> str:
    """Return a one-line summary from the node docstring."""
    doc = ast.get_docstring(node) or ""
    return doc.strip().splitlines()[0] if doc.strip() else "-"


def parse_file(path: Path) -> Tuple[str, List[Tuple[str, str, str]]]:
    """Parse one file and extract function / class summaries."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    module_doc = summarize_doc(tree)
    items: List[Tuple[str, str, str]] = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            items.append(("class", node.name, summarize_doc(node)))
            for child in node.body:
                if isinstance(child, ast.FunctionDef):
                    items.append(("method", f"{node.name}.{child.name}", summarize_doc(child)))
        elif isinstance(node, ast.FunctionDef):
            items.append(("function", node.name, summarize_doc(node)))
    return module_doc, items


def main() -> None:
    """Generate wiki/function_index.md."""
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# Function Index", "", "本文件由 `tools/generate_wiki.py` 自动生成。", ""]
    for path in iter_python_files():
        rel = path.relative_to(ROOT).as_posix()
        module_doc, items = parse_file(path)
        lines.append(f"## `{rel}`")
        lines.append("")
        lines.append(f"- Module: {module_doc}")
        lines.append("")
        if items:
            lines.append("| Kind | Name | Summary |")
            lines.append("|---|---|---|")
            for kind, name, summary in items:
                lines.append(f"| {kind} | `{name}` | {summary} |")
            lines.append("")
    OUT_PATH.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
