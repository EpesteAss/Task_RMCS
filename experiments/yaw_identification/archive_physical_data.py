#!/usr/bin/env python3
"""Archive the 2026-10-14 physical captures under data/ with checksums."""
from __future__ import annotations

import csv
import hashlib
import shutil
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "physical_data"
DESTINATION = HERE / "data" / "physical_20261014"


def digest(path: Path) -> str:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def main() -> None:
    captures = sorted(path for path in SOURCE.glob("20261014_*") if path.is_dir())
    if not captures:
        raise SystemExit("No captures for 2026-10-14 found")
    DESTINATION.mkdir(parents=True, exist_ok=True)
    manifest = []
    for capture in captures:
        for original in sorted(path for path in capture.rglob("*") if path.is_file()):
            relative = original.relative_to(SOURCE)
            archived = DESTINATION / relative
            archived.parent.mkdir(parents=True, exist_ok=True)
            if not archived.exists() or digest(archived) != digest(original):
                shutil.copy2(original, archived)
            original_hash = digest(original)
            if digest(archived) != original_hash:
                raise RuntimeError(f"Archive mismatch: {relative}")
            manifest.append((relative.as_posix(), original.stat().st_size, original_hash))
    with (DESTINATION / "MANIFEST.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("relative_path", "bytes", "sha256"))
        writer.writerows(manifest)
    print(f"Archived {len(captures)} capture directories and {len(manifest)} files to {DESTINATION}")


if __name__ == "__main__":
    main()
