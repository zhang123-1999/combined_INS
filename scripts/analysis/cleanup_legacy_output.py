from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def rel_to_base(path: Path, base: Path) -> str:
    return path.resolve().relative_to(base.resolve()).as_posix()


def common_display_root(*paths: Path) -> Path:
    resolved = [str(path.resolve()) for path in paths]
    return Path(os.path.commonpath(resolved))


def collect_candidates(
    output_root: Path,
    keep_names: set[str],
    include_prefixes: tuple[str, ...] = (),
) -> list[dict[str, str]]:
    candidates: list[dict[str, str]] = []
    display_root = output_root.resolve().parent
    for child in sorted(output_root.iterdir(), key=lambda item: item.name.lower()):
        if child.name in keep_names:
            continue
        if include_prefixes and not any(child.name.startswith(prefix) for prefix in include_prefixes):
            continue
        candidates.append(
            {
                "path": rel_to_base(child, display_root),
                "name": child.name,
                "kind": "dir" if child.is_dir() else "file",
            }
        )
    return candidates


def delete_candidate(output_root: Path, rel_path: str) -> None:
    path = REPO_ROOT / rel_path
    if not path.exists():
        return
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def archive_candidate(output_root: Path, rel_name: str, archive_root: Path) -> dict[str, str]:
    source_path = (output_root / rel_name).resolve()
    if not source_path.exists():
        raise FileNotFoundError(f"missing legacy output entry: {source_path}")

    archive_target = (archive_root / rel_name).resolve()
    if archive_target.exists():
        raise FileExistsError(f"archive target already exists: {archive_target}")

    archive_target.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(source_path), str(archive_target))

    display_root = common_display_root(output_root, archive_root)
    return {
        "name": rel_name,
        "source_path": rel_to_base(source_path, display_root),
        "archive_path": rel_to_base(archive_target, display_root),
        "kind": "dir" if archive_target.is_dir() else "file",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Archive legacy output entries while keeping the canonical standardized result folders."
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("output"),
        help="Output root relative to repo root.",
    )
    parser.add_argument(
        "--keep",
        action="append",
        default=["data2_baseline_current"],
        help="Top-level names under output/ to keep. Can be specified multiple times.",
    )
    parser.add_argument(
        "--include-prefix",
        action="append",
        default=["data2_"],
        help="Only consider top-level output entries whose names start with one of these prefixes.",
    )
    parser.add_argument(
        "--archive-root",
        type=Path,
        default=Path("archive/output_legacy/20260326-baseline-finalization"),
        help="Archive destination root relative to repo root.",
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=Path("output/data2_baseline_current/artifacts/legacy_cleanup_report.json"),
        help="Path of the archive report relative to repo root.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually archive legacy entries. Without this flag the script only reports candidates.",
    )
    args = parser.parse_args()
    args.output_root = (REPO_ROOT / args.output_root).resolve()
    args.archive_root = (REPO_ROOT / args.archive_root).resolve()
    args.report_path = (REPO_ROOT / args.report_path).resolve()
    args.keep = sorted(set(args.keep))
    args.include_prefix = tuple(dict.fromkeys(args.include_prefix))
    return args


def main() -> None:
    args = parse_args()
    if not args.output_root.exists():
        raise FileNotFoundError(f"missing output root: {args.output_root}")

    keep_names = set(args.keep)
    candidates = collect_candidates(args.output_root, keep_names, include_prefixes=args.include_prefix)
    display_root = common_display_root(args.output_root, args.archive_root, args.report_path)

    report = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "output_root": rel_to_base(args.output_root, display_root),
        "archive_root": rel_to_base(args.archive_root, display_root),
        "keep_names": args.keep,
        "include_prefixes": list(args.include_prefix),
        "execute": bool(args.execute),
        "candidate_count": len(candidates),
        "candidates": candidates,
        "archived": [],
    }

    print("Legacy output candidates:")
    for item in candidates:
        print(f"  - {item['kind']}: {item['path']}")

    if args.execute:
        for item in candidates:
            archived = archive_candidate(args.output_root, item["name"], args.archive_root)
            report["archived"].append(archived)
        print(f"Archived {len(report['archived'])} legacy entries.")
    else:
        print("Dry run only. No files archived.")

    args.report_path.parent.mkdir(parents=True, exist_ok=True)
    args.report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_to_base(args.report_path, display_root))


if __name__ == "__main__":
    main()
