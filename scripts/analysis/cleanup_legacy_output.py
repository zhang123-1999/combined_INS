from __future__ import annotations

import argparse
import datetime as dt
import json
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def rel_from_root(path: Path) -> str:
    return path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()


def collect_candidates(output_root: Path, keep_names: set[str]) -> list[dict[str, str]]:
    candidates: list[dict[str, str]] = []
    for child in sorted(output_root.iterdir(), key=lambda item: item.name.lower()):
        if child.name in keep_names:
            continue
        candidates.append(
            {
                "path": rel_from_root(child),
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Delete legacy output entries while keeping canonical standardized result folders.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("output"),
        help="Output root relative to repo root.",
    )
    parser.add_argument(
        "--keep",
        action="append",
        default=["data2_eskf_baseline"],
        help="Top-level names under output/ to keep. Can be specified multiple times.",
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=Path("output/data2_eskf_baseline/artifacts/legacy_cleanup_report.json"),
        help="Path of the deletion report relative to repo root.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually delete legacy entries. Without this flag the script only reports candidates.",
    )
    args = parser.parse_args()
    args.output_root = (REPO_ROOT / args.output_root).resolve()
    args.report_path = (REPO_ROOT / args.report_path).resolve()
    args.keep = sorted(set(args.keep))
    return args


def main() -> None:
    args = parse_args()
    if not args.output_root.exists():
        raise FileNotFoundError(f"missing output root: {args.output_root}")

    keep_names = set(args.keep)
    candidates = collect_candidates(args.output_root, keep_names)

    report = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "output_root": rel_from_root(args.output_root),
        "keep_names": args.keep,
        "execute": bool(args.execute),
        "candidate_count": len(candidates),
        "candidates": candidates,
        "deleted": [],
    }

    print("Legacy output candidates:")
    for item in candidates:
        print(f"  - {item['kind']}: {item['path']}")

    if args.execute:
        for item in candidates:
            delete_candidate(args.output_root, item["path"])
            report["deleted"].append(item["path"])
        print(f"Deleted {len(report['deleted'])} legacy entries.")
    else:
        print("Dry run only. No files deleted.")

    args.report_path.parent.mkdir(parents=True, exist_ok=True)
    args.report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(args.report_path))


if __name__ == "__main__":
    main()
