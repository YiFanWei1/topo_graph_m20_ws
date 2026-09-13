#!/usr/bin/env python3
"""Migrate copied Route3D topology JSON files to M20 authoring defaults.

Only topology JSON is touched.  A byte-for-byte backup is written below
``data/.m20_migration_backup`` before the first change to each source file.
Mixed/manual values are preserved; legacy locomotionMode is removed because
M20 has no gait selection.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any


def _is_web_authored(items: list[dict[str, Any]]) -> bool:
    for item in items:
        meta = item.get("meta", {})
        source = str(meta.get("source", "")).lower()
        if "web" in source or "manual" in source:
            return True
    return False


def migrate_document(document: dict[str, Any]) -> dict[str, Any]:
    vertices = list(document.get("vertices", {}).values())
    edges = list(document.get("edges", {}).values())
    result: dict[str, Any] = {
        "vertices": len(vertices),
        "edges": len(edges),
        "obstacle_mode_changed": 0,
        "align_final_yaw_changed": 0,
        "locomotion_mode_removed": 0,
    }

    edge_values = [edge.get("meta", {}).get("obstacleMode") for edge in edges]
    auto_all_zero = bool(edges) and all(value == 0 for value in edge_values) and not _is_web_authored(edges)
    if auto_all_zero:
        for edge in edges:
            edge.setdefault("meta", {})["obstacleMode"] = 1
        result["obstacle_mode_changed"] = len(edges)

    align_values = [vertex.get("alignFinalYaw") for vertex in vertices]
    auto_all_true = bool(vertices) and all(value is True for value in align_values) and not _is_web_authored(vertices)
    if auto_all_true:
        for vertex in vertices:
            vertex["alignFinalYaw"] = False
        result["align_final_yaw_changed"] = len(vertices)

    for edge in edges:
        meta = edge.get("meta")
        if isinstance(meta, dict) and "locomotionMode" in meta:
            del meta["locomotionMode"]
            result["locomotion_mode_removed"] += 1
    result["changed"] = any(result[key] for key in (
        "obstacle_mode_changed", "align_final_yaw_changed", "locomotion_mode_removed"))
    return result


def _atomic_write(path: Path, document: dict[str, Any]) -> None:
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=workspace / "data")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--report", type=Path, default=workspace / "validation" / "m20_topology_migration.json")
    args = parser.parse_args()
    data_root = args.data_root.expanduser().resolve()
    backup_root = data_root / ".m20_migration_backup"

    reports: list[dict[str, Any]] = []
    for path in sorted(data_root.rglob("topoGraph_data.json")):
        if backup_root in path.parents:
            continue
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        result = migrate_document(document)
        result["file"] = str(path.relative_to(data_root))
        reports.append(result)
        if args.apply and result["changed"]:
            backup_path = backup_root / path.relative_to(data_root)
            backup_path.parent.mkdir(parents=True, exist_ok=True)
            if not backup_path.exists():
                shutil.copy2(path, backup_path)
            _atomic_write(path, document)

    report = {
        "data_root": str(data_root),
        "applied": args.apply,
        "backup_root": str(backup_root),
        "files_scanned": len(reports),
        "files_changed": sum(bool(item["changed"]) for item in reports),
        "totals": {
            key: sum(int(item[key]) for item in reports)
            for key in ("obstacle_mode_changed", "align_final_yaw_changed", "locomotion_mode_removed")
        },
        "files": reports,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    _atomic_write(args.report, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
