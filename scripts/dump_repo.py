#!/usr/bin/env python3
"""
dump_repo.py  —  concatenate all source files in a repo into one text file.

Usage:
    python dump_repo.py                        # dumps current directory
    python dump_repo.py /path/to/repo          # dumps specified path
    python dump_repo.py /path/to/repo -o out.txt

Output format:
    ================================================================================
    FILE: src/conceptual_design/sizing.py
    ================================================================================
    <file contents>

"""

import argparse
import os
import sys
from pathlib import Path

# File extensions to include
INCLUDE_EXTS = {
    ".py", ".ipynb", ".md", ".txt", ".yaml", ".yml",
    ".toml", ".cfg", ".ini", ".json", ".rst",
}

# Directories to always skip
SKIP_DIRS = {
    ".git", "__pycache__", ".ipynb_checkpoints",
    "node_modules", ".venv", "venv", "env",
    ".mypy_cache", ".pytest_cache", "dist", "build",
    ".tox", ".eggs", "*.egg-info",
}

# Files to always skip
SKIP_FILES = {
    ".DS_Store", "Thumbs.db", "*.pyc",
}

SEPARATOR = "=" * 80


def should_skip_dir(name: str) -> bool:
    return name in SKIP_DIRS or name.endswith(".egg-info")


def should_skip_file(name: str) -> bool:
    return name in SKIP_FILES or name.endswith(".pyc")


def collect_files(root: Path) -> list[Path]:
    files = []
    for dirpath, dirnames, filenames in os.walk(root):
        # Prune skipped dirs in-place so os.walk doesn't descend into them
        dirnames[:] = [d for d in sorted(dirnames) if not should_skip_dir(d)]
        for fname in sorted(filenames):
            if should_skip_file(fname):
                continue
            fpath = Path(dirpath) / fname
            if fpath.suffix in INCLUDE_EXTS or fpath.name in {"Makefile", "Dockerfile"}:
                files.append(fpath)
    return files


def dump(root: Path, output: Path) -> None:
    files = collect_files(root)
    if not files:
        print("No matching files found.", file=sys.stderr)
        sys.exit(1)

    with output.open("w", encoding="utf-8") as out:
        out.write(f"REPO DUMP  —  root: {root.resolve()}\n")
        out.write(f"Files included: {len(files)}\n")
        out.write(SEPARATOR + "\n\n")

        for fpath in files:
            rel = fpath.relative_to(root)
            out.write(f"\n{SEPARATOR}\n")
            out.write(f"FILE: {rel}\n")
            out.write(f"{SEPARATOR}\n")
            try:
                content = fpath.read_text(encoding="utf-8", errors="replace")
                out.write(content)
                if not content.endswith("\n"):
                    out.write("\n")
            except Exception as e:
                out.write(f"[ERROR reading file: {e}]\n")

    print(f"Dumped {len(files)} files → {output}")
    for f in files:
        print(f"  {f.relative_to(root)}")


def main():
    parser = argparse.ArgumentParser(description="Dump repo source files to a single text file.")
    parser.add_argument("root", nargs="?", default=".", help="Repo root directory (default: current dir)")
    parser.add_argument("-o", "--output", default="repo_dump.txt", help="Output file (default: repo_dump.txt)")
    parser.add_argument("--ext", nargs="*", help="Extra extensions to include, e.g. --ext .sh .R")
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    if not root.is_dir():
        print(f"Error: {root} is not a directory.", file=sys.stderr)
        sys.exit(1)

    if args.ext:
        for e in args.ext:
            INCLUDE_EXTS.add(e if e.startswith(".") else f".{e}")

    output = Path(args.output)
    dump(root, output)


if __name__ == "__main__":
    main()