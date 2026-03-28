import importlib.util
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "cleanup_legacy_output.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing cleanup helper: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("cleanup_legacy_output", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_collect_candidates_supports_prefix_filters_and_keep_list(tmp_path):
    module = load_module()
    output_root = tmp_path / "output"
    (output_root / "data2_baseline_current").mkdir(parents=True)
    (output_root / "data2_staged_g5_best_rerun_r1_20260326").mkdir()
    (output_root / "data2_bgz_route_decomposition_r1_20260325").mkdir()
    (output_root / "review").mkdir()

    candidates = module.collect_candidates(
        output_root,
        {"data2_baseline_current"},
        include_prefixes=("data2_",),
    )

    assert [item["name"] for item in candidates] == [
        "data2_bgz_route_decomposition_r1_20260325",
        "data2_staged_g5_best_rerun_r1_20260326",
    ]


def test_archive_candidate_moves_output_into_archive_root(tmp_path):
    module = load_module()
    output_root = tmp_path / "output"
    archive_root = tmp_path / "archive" / "output_legacy" / "20260326-baseline-finalization"
    source_dir = output_root / "data2_staged_g5_best_rerun_r1_20260326"
    source_dir.mkdir(parents=True)
    (source_dir / "summary.md").write_text("legacy", encoding="utf-8")

    archived = module.archive_candidate(output_root, "data2_staged_g5_best_rerun_r1_20260326", archive_root)

    assert not source_dir.exists()
    assert (
        archive_root / "data2_staged_g5_best_rerun_r1_20260326" / "summary.md"
    ).read_text(encoding="utf-8") == "legacy"
    assert archived["source_path"] == "output/data2_staged_g5_best_rerun_r1_20260326"
    assert (
        archived["archive_path"]
        == "archive/output_legacy/20260326-baseline-finalization/data2_staged_g5_best_rerun_r1_20260326"
    )
