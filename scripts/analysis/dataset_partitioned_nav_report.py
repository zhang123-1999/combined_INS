from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import pandas as pd

from scripts.analysis.interactive_nav_report import CaseSpec, SectionResult, SectionSpec, build_report_from_sections
from scripts.analysis.odo_nhc_update_sweep import load_yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
REPORT_TITLE = "INS/GNSS/ODO/NHC组合导航结果"
RAD2DEG = 180.0 / math.pi

STATE_PLOT_ORDER = [
    "01_trajectory_altitude.png",
    "02_velocity_NED.png",
    "02b_velocity_vehicle_v.png",
    "03_attitude.png",
    "04_gyro_bias.png",
    "05_accel_bias.png",
    "06_gyro_scale_factor.png",
    "07_accel_scale_factor.png",
    "08_mount_angles.png",
    "09_lever_arm.png",
    "10_gnss_lever_arm.png",
    "11_ecef_position.png",
]
STATE_PLOT_LABELS = {
    "01_trajectory_altitude.png": "轨迹与高度",
    "02_velocity_NED.png": "n系速度",
    "02b_velocity_vehicle_v.png": "v系速度",
    "03_attitude.png": "姿态角",
    "04_gyro_bias.png": "陀螺零偏",
    "05_accel_bias.png": "加计零偏",
    "06_gyro_scale_factor.png": "陀螺比例因子",
    "07_accel_scale_factor.png": "加计比例因子",
    "08_mount_angles.png": "安装角",
    "09_lever_arm.png": "ODO杆臂",
    "10_gnss_lever_arm.png": "GNSS杆臂",
    "11_ecef_position.png": "ECEF位置",
}
CASE_COLORS = {
    "full": "#2E75D6",
    "eskf_best": "#1FA187",
    "inekf_best": "#C0392B",
    "outage_eskf": "#E67E22",
    "outage_inekf": "#27AE60",
}

STATE_TABLE_COLUMNS = [
    "状态块",
    "维数",
    "初始标准差",
    "过程噪声",
    "算法差异",
    "说明",
]

OBS_TABLE_COLUMNS = [
    "信号源",
    "原始数据可用性",
    "量测噪声",
    "公共更新节奏",
    "场景差异",
    "说明",
]


@dataclass(frozen=True)
class ReaderTableSpec:
    title: str
    dataframe: pd.DataFrame
    note_html: str = ""
    table_kind: str = "default"


@dataclass(frozen=True)
class IntroSectionSpec:
    title: str
    body_html: str = ""
    tables: tuple[ReaderTableSpec, ...] = ()


@dataclass(frozen=True)
class StateGallerySpec:
    case_id: str
    label: str
    sol_path: str
    config_path: str
    image_dir: str
    dedupe_key: str


@dataclass(frozen=True)
class DatasetSectionSpec:
    exp_id: str
    title: str
    subtitle: str
    dataset_id: str
    family_id: str
    case_specs: tuple[CaseSpec, ...]
    state_galleries: tuple[StateGallerySpec, ...]


@dataclass(frozen=True)
class DatasetGroupSpec:
    group_id: str
    title: str


@dataclass(frozen=True)
class FilterProfile:
    dataset_id: str
    method_id: str
    method_label: str
    config: dict[str, Any]
    init: dict[str, Any]
    constraints: dict[str, Any]
    noise: dict[str, Any]
    p0_diag: list[float]
    p0_source_desc: str


@dataclass
class DatasetReportPage:
    page_id: str
    group_id: str
    nav_label: str
    title: str
    subtitle: str
    intro_sections: tuple[IntroSectionSpec, ...] = ()
    section_result: SectionResult | None = None
    analysis_html: str = ""
    state_galleries: tuple[StateGallerySpec, ...] = ()


@dataclass
class DatasetPartitionedReport:
    report_title: str
    groups: tuple[DatasetGroupSpec, ...]
    pages: list[DatasetReportPage]
    manifest_path: Path


def _latest_cases_manifest() -> Path:
    candidates = sorted((REPO_ROOT / "output" / "review").glob("EXP-*-dataset-report-cases-r*/manifest.json"))
    if not candidates:
        raise FileNotFoundError("missing canonical cases manifest under output/review/EXP-*-dataset-report-cases-r*/manifest.json")
    return max(candidates, key=lambda path: (path.stat().st_mtime, path.parent.name))


def _load_manifest() -> tuple[Path, dict[str, Any]]:
    manifest_path = _latest_cases_manifest()
    with manifest_path.open("r", encoding="utf-8") as f:
        manifest = json.load(f)
    return manifest_path, manifest


def _case_map(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    cases = manifest.get("cases") or []
    mapping = {str(row["case_id"]): row for row in cases}
    if not mapping:
        raise RuntimeError("canonical cases manifest has no cases")
    return mapping


def _case_row(case_rows: dict[str, dict[str, Any]], case_id: str) -> dict[str, Any]:
    row = case_rows.get(case_id)
    if row is None:
        raise KeyError(f"missing case in manifest: {case_id}")
    return row


def _float(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if value is None:
        return float("nan")
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _fmt(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "nan"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "nan"
    if not math.isfinite(number):
        return "nan"
    return f"{number:.{digits}f}"


def _analysis_card(title: str, items: list[str]) -> str:
    body = "".join(f"<li>{item}</li>" for item in items)
    return (
        '<div class="analysis-card">'
        f'<div class="analysis-title">{title}</div>'
        f'<ul class="analysis-list">{body}</ul>'
        "</div>"
    )


def _as_float_list(values: Any, length: int, default: float) -> list[float]:
    if isinstance(values, (list, tuple)) and len(values) == length:
        return [float(v) for v in values]
    return [float(default)] * length


def _std_equivalent_p0_diag(init_cfg: dict[str, Any]) -> list[float]:
    std_pos = _as_float_list(init_cfg.get("std_pos"), 3, 0.1)
    std_vel = _as_float_list(init_cfg.get("std_vel"), 3, 0.1)
    std_att = _as_float_list(init_cfg.get("std_att"), 3, 0.1)
    std_ba = _as_float_list(init_cfg.get("std_ba"), 3, 0.01)
    std_bg = _as_float_list(init_cfg.get("std_bg"), 3, 0.01)
    std_sg = _as_float_list(init_cfg.get("std_sg"), 3, 0.001)
    std_sa = _as_float_list(init_cfg.get("std_sa"), 3, 0.001)
    std_odo_scale = float(init_cfg.get("std_odo_scale", 0.05))
    std_mount_roll = float(init_cfg.get("std_mounting_roll", 0.5))
    std_mount_pitch = float(init_cfg.get("std_mounting_pitch", 0.5))
    std_mount_yaw = float(init_cfg.get("std_mounting_yaw", 0.5))
    std_lever = _as_float_list(init_cfg.get("std_lever_arm"), 3, 0.1)
    std_gnss_lever = _as_float_list(init_cfg.get("std_gnss_lever_arm"), 3, 0.1)

    deg2rad = math.pi / 180.0
    diag: list[float] = []
    diag.extend(v * v for v in std_pos)
    diag.extend(v * v for v in std_vel)
    diag.extend((v * deg2rad) * (v * deg2rad) for v in std_att)
    diag.extend(v * v for v in std_ba)
    diag.extend(v * v for v in std_bg)
    diag.extend(v * v for v in std_sg)
    diag.extend(v * v for v in std_sa)
    diag.append(std_odo_scale * std_odo_scale)
    diag.append((std_mount_roll * deg2rad) * (std_mount_roll * deg2rad))
    diag.append((std_mount_pitch * deg2rad) * (std_mount_pitch * deg2rad))
    diag.append((std_mount_yaw * deg2rad) * (std_mount_yaw * deg2rad))
    diag.extend(v * v for v in std_lever)
    diag.extend(v * v for v in std_gnss_lever)
    return diag


def _approx_same(a: list[float], b: list[float]) -> bool:
    if len(a) != len(b):
        return False
    for lhs, rhs in zip(a, b):
        tol = max(1.0e-12, 1.0e-6 * max(abs(lhs), abs(rhs), 1.0))
        if abs(lhs - rhs) > tol:
            return False
    return True


def _effective_p0_diag(cfg: dict[str, Any]) -> tuple[list[float], str]:
    fusion_cfg = cfg.get("fusion") or {}
    init_cfg = fusion_cfg.get("init") or {}
    std_diag = _std_equivalent_p0_diag(init_cfg)
    custom_diag = init_cfg.get("P0_diag")
    if isinstance(custom_diag, (list, tuple)) and len(custom_diag) == len(std_diag):
        diag = [float(v) for v in custom_diag]
        if _approx_same(diag, std_diag):
            return diag, "由设定的初始标准差平方得到"
        return diag, "由对角初始协方差直接给定"
    return std_diag, "由设定的初始标准差平方得到"


def _display_method(method_id: str) -> str:
    return "InEKF" if method_id == "true_iekf" else method_id.upper()


def _display_rate(label: Any) -> str:
    text = str(label).strip()
    lowered = text.lower()
    if lowered == "raw":
        return "原始频率"
    match = re.fullmatch(r"([0-9]*\.?[0-9]+)\s*hz", lowered)
    if match:
        return f"{float(match.group(1)):g} Hz"
    return text.replace("true_iekf", "InEKF")


def _display_case_label(case_id: str) -> str:
    suffix_map = {
        "full_gnss_eskf": "全程GNSS ESKF",
        "gnss30_eskf_best": "GNSS30 ESKF",
        "gnss30_true_iekf_best": "GNSS30 InEKF",
        "gnss_outage_eskf_best": "周期开断 ESKF",
        "gnss_outage_true_iekf_best": "周期开断 InEKF",
    }
    for suffix, label in suffix_map.items():
        if case_id.endswith(suffix):
            return label
    return case_id.replace("true_iekf", "InEKF")


def _display_case_id(case_id: str) -> str:
    return case_id.replace("true_iekf", "inekf")


def _make_case_spec(row: dict[str, Any], color: str) -> CaseSpec:
    return CaseSpec(
        case_id=_display_case_id(str(row["case_id"])),
        label=_display_case_label(str(row["case_id"])),
        sol_path=str(row["sol_path"]),
        config_path=str(row["config_path"]),
        color=color,
    )


def _make_gallery_spec(row: dict[str, Any]) -> StateGallerySpec:
    return StateGallerySpec(
        case_id=str(row["case_id"]),
        label=_display_case_label(str(row["case_id"])),
        sol_path=str(row["sol_path"]),
        config_path=str(row["config_path"]),
        image_dir=str(row["image_dir"]),
        dedupe_key=str((REPO_ROOT / str(row["sol_path"])).resolve()),
    )


def _load_filter_profile(row: dict[str, Any]) -> FilterProfile:
    cfg = load_yaml((REPO_ROOT / str(row["config_path"])).resolve())
    fusion_cfg = cfg.get("fusion") or {}
    init_cfg = fusion_cfg.get("init") or {}
    constraints_cfg = fusion_cfg.get("constraints") or {}
    noise_cfg = fusion_cfg.get("noise") or {}
    p0_diag, p0_source_desc = _effective_p0_diag(cfg)
    method_id = str(row.get("method", "eskf"))
    return FilterProfile(
        dataset_id=str(row["dataset_id"]),
        method_id=method_id,
        method_label=_display_method(method_id),
        config=cfg,
        init=init_cfg,
        constraints=constraints_cfg,
        noise=noise_cfg,
        p0_diag=p0_diag,
        p0_source_desc=p0_source_desc,
    )


def _format_number(value: float, digits: int = 3) -> str:
    if not math.isfinite(value):
        return "nan"
    magnitude = abs(value)
    if magnitude == 0.0:
        return "0"
    if magnitude >= 1000.0 or magnitude < 1.0e-3:
        return f"{value:.{digits}e}"
    return f"{value:.{digits}f}".rstrip("0").rstrip(".")


def _format_series(values: list[float], unit: str, axis_labels: tuple[str, ...] | None = None, *, scientific: bool = False) -> str:
    if not values:
        return "—"
    formatter = (lambda v: f"{v:.3e}") if scientific else _format_number
    if len(values) == 1:
        return f"{formatter(values[0])} {unit}".strip()
    if _approx_same(values, [values[0]] * len(values)):
        return f"3轴相同：{formatter(values[0])} {unit}".strip()
    labels = axis_labels or tuple(f"轴{i + 1}" for i in range(len(values)))
    joined = " / ".join(formatter(v) for v in values)
    prefix = "/".join(labels)
    return f"{prefix} = {joined} {unit}".strip()


def _format_std_from_diag(diag_values: list[float], mode: str, axis_labels: tuple[str, ...] | None = None) -> str:
    std_values = [math.sqrt(max(v, 0.0)) for v in diag_values]
    if mode == "angle":
        return _format_series([v * RAD2DEG for v in std_values], "deg", axis_labels)
    if mode == "scale":
        return _format_series([v * 1.0e6 for v in std_values], "ppm", axis_labels)
    unit_map = {
        "position": "m",
        "velocity": "m/s",
        "accel_bias": "m/s²",
        "gyro_bias": "rad/s",
    }
    return _format_series(std_values, unit_map[mode], axis_labels)


def _format_diag(diag_values: list[float], mode: str, axis_labels: tuple[str, ...] | None = None) -> str:
    unit_map = {
        "position": "m²",
        "velocity": "(m/s)²",
        "angle": "rad²",
        "accel_bias": "(m/s²)²",
        "gyro_bias": "(rad/s)²",
        "scale": "ratio²",
    }
    return _format_series(diag_values, unit_map[mode], axis_labels, scientific=True)


def _shared_text(eskf_value: str, inekf_value: str) -> str:
    return eskf_value if eskf_value == inekf_value else f"ESKF：{eskf_value}；InEKF：{inekf_value}"


def _difference_text(label: str, eskf_value: str, inekf_value: str) -> str:
    if eskf_value == inekf_value:
        return ""
    return f"{label}：ESKF {eskf_value}；InEKF {inekf_value}"


def _merge_difference_items(items: list[str]) -> str:
    filtered = [item for item in items if item]
    return "；".join(filtered) if filtered else "—"


def _scene_pair_text(eskf_value: str, inekf_value: str) -> str:
    return eskf_value if eskf_value == inekf_value else f"ESKF {eskf_value}，InEKF {inekf_value}"


def _noise_angle_text(value: float) -> str:
    return f"{_format_number(value)} rad/√Hz（约 {_format_number(value * RAD2DEG, 4)} deg/√Hz）"


def _noise_scale_text(value: float) -> str:
    return f"{_format_number(value * 1.0e6)} ppm/√Hz"


def _imu_process_noise_text(profile: FilterProfile) -> str:
    return (
        f"σ_a = {_format_number(float(profile.noise.get('sigma_acc', 0.0)))} m/s²/√Hz；"
        f"σ_ω = {_format_number(float(profile.noise.get('sigma_gyro', 0.0)))} rad/s/√Hz"
    )


def _state_process_text(profile: FilterProfile, key: str, unit: str, formatter: Callable[[float], str] | None = None) -> str:
    formatter = formatter or (lambda value: f"{_format_number(value)} {unit}".strip())
    return formatter(float(profile.noise.get(key, 0.0)))


STATE_BLOCK_SPECS: tuple[dict[str, Any], ...] = (
    {
        "label": "位置 p",
        "dim": 3,
        "indices": (0, 1, 2),
        "mode": "position",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _imu_process_noise_text(profile),
        "note": lambda _: "位置误差由 IMU 传播累积，并通过 GNSS / ODO / NHC 等外部观测逐步校正。",
    },
    {
        "label": "速度 v",
        "dim": 3,
        "indices": (3, 4, 5),
        "mode": "velocity",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _imu_process_noise_text(profile),
        "note": lambda _: "速度状态直接承接 ODO、NHC 与 GNSS 速度信息的校正作用。",
    },
    {
        "label": "姿态 att",
        "dim": 3,
        "indices": (6, 7, 8),
        "mode": "angle",
        "axis_labels": ("roll", "pitch", "yaw"),
        "estimated": "是",
        "process": lambda profile, _: _imu_process_noise_text(profile),
        "note": lambda _: "姿态误差按三维小角度状态建模；该块与 v 系航向误差演化直接相关。",
    },
    {
        "label": "加计零偏 ba",
        "dim": 3,
        "indices": (9, 10, 11),
        "mode": "accel_bias",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_ba", "m/s²/√Hz"),
        "note": lambda _: "零偏采用随机游走/慢变模型，与 IMU 长时漂移校正相关。",
    },
    {
        "label": "陀螺零偏 bg",
        "dim": 3,
        "indices": (12, 13, 14),
        "mode": "gyro_bias",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_bg", "rad/s/√Hz"),
        "note": lambda _: "该状态与航向漂移高度相关，是 sparse-GNSS 条件下的重要慢变误差源。",
    },
    {
        "label": "陀螺比例因子 sg",
        "dim": 3,
        "indices": (15, 16, 17),
        "mode": "scale",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_sg", "", _noise_scale_text),
        "note": lambda _: "以无量纲比例因子表示；表中标准差换算为 ppm 便于阅读。",
    },
    {
        "label": "加计比例因子 sa",
        "dim": 3,
        "indices": (18, 19, 20),
        "mode": "scale",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_sa", "", _noise_scale_text),
        "note": lambda _: "以无量纲比例因子表示；表中标准差换算为 ppm 便于阅读。",
    },
    {
        "label": "ODO 比例因子",
        "dim": 1,
        "indices": (21,),
        "mode": "scale",
        "axis_labels": None,
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_odo_scale", "", _noise_scale_text),
        "note": lambda _: "无量纲比例因子，直接作用于 ODO 速度观测通道。",
    },
    {
        "label": "安装角 roll",
        "dim": 1,
        "indices": (22,),
        "mode": "angle",
        "axis_labels": None,
        "estimated": "保留但默认不主动更新",
        "process": lambda profile, _: _state_process_text(profile, "sigma_mounting_roll", "", _noise_angle_text),
        "note": lambda _: "状态仍保留在 31 维向量中，但当前主 ODO/NHC 观测 Jacobian 只显式作用于 pitch / yaw，因此该项默认不主动估计/更新，按固定量处理。",
    },
    {
        "label": "安装角 pitch",
        "dim": 1,
        "indices": (23,),
        "mode": "angle",
        "axis_labels": None,
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_mounting_pitch", "", _noise_angle_text),
        "note": lambda _: "当前主 ODO/NHC 观测链路对该项有显式 Jacobian，是主要安装角估计自由度之一。",
    },
    {
        "label": "安装角 yaw",
        "dim": 1,
        "indices": (24,),
        "mode": "angle",
        "axis_labels": None,
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_mounting_yaw", "", _noise_angle_text),
        "note": lambda _: "当前主 ODO/NHC 观测链路对该项有显式 Jacobian，是车体航向相关的关键安装角自由度。",
    },
    {
        "label": "ODO 杆臂",
        "dim": 3,
        "indices": (25, 26, 27),
        "mode": "position",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_lever_arm", "m/√Hz"),
        "note": lambda _: "通过 ODO / NHC 旋转耦合项进入观测模型，对转弯与角速度激励较敏感。",
    },
    {
        "label": "GNSS 杆臂",
        "dim": 3,
        "indices": (28, 29, 30),
        "mode": "position",
        "axis_labels": ("x", "y", "z"),
        "estimated": "是",
        "process": lambda profile, _: _state_process_text(profile, "sigma_gnss_lever_arm", "m/√Hz"),
        "note": lambda dataset_id: (
            "当前数据集同时由 GNSS 位置与 GNSS 速度链路共同约束该状态。"
            if dataset_id == "data4"
            else "当前数据集仅由 GNSS 位置链路直接约束该状态；由于没有 GNSS 速度信息，不存在额外的速度侧直接约束。"
        ),
    },
)


def _state_noise_table(dataset_id: str, eskf: FilterProfile, inekf: FilterProfile) -> pd.DataFrame:
    rows: list[dict[str, str]] = []
    for spec in STATE_BLOCK_SPECS:
        indices = tuple(int(idx) for idx in spec["indices"])
        eskf_diag = [float(eskf.p0_diag[idx]) for idx in indices]
        inekf_diag = [float(inekf.p0_diag[idx]) for idx in indices]
        eskf_std = _format_std_from_diag(eskf_diag, str(spec["mode"]), spec.get("axis_labels"))
        inekf_std = _format_std_from_diag(inekf_diag, str(spec["mode"]), spec.get("axis_labels"))
        eskf_process = str(spec["process"](eskf, dataset_id))
        inekf_process = str(spec["process"](inekf, dataset_id))
        diff_items = [
            _difference_text("初始标准差", eskf_std, inekf_std),
            _difference_text("过程噪声", eskf_process, inekf_process),
        ]
        rows.append(
            {
                "状态块": str(spec["label"]),
                "维数": str(spec["dim"]),
                "初始标准差": eskf_std if eskf_std == inekf_std else "见算法差异",
                "过程噪声": eskf_process if eskf_process == inekf_process else "见算法差异",
                "算法差异": _merge_difference_items(diff_items),
                "说明": str(spec["note"](dataset_id)),
            }
        )
    return pd.DataFrame(rows, columns=STATE_TABLE_COLUMNS)


def _dataset_parameter_intro(dataset_id: str, eskf: FilterProfile, inekf: FilterProfile) -> str:
    corr_time = int(round(float(eskf.noise.get("markov_corr_time", 0.0))))
    return (
        "<p class=\"section-copy\">"
        f"{dataset_id} 的表格已把 ESKF 与 InEKF 的相同设置合并展示；仅把不同项写入“算法差异”或“场景差异”列。"
        f"随机参数相关时间统一设为 <strong>{corr_time} s</strong>。"
        "</p>"
    )


def _observation_design_table(dataset_id: str, case_rows: dict[str, dict[str, Any]], eskf: FilterProfile, inekf: FilterProfile) -> pd.DataFrame:
    full_row = _case_row(case_rows, f"{dataset_id}_full_gnss_eskf")
    gnss30_eskf_row = _case_row(case_rows, f"{dataset_id}_gnss30_eskf_best")
    gnss30_inekf_row = _case_row(case_rows, f"{dataset_id}_gnss30_true_iekf_best")
    outage_eskf_row = _case_row(case_rows, f"{dataset_id}_gnss_outage_eskf_best")
    outage_inekf_row = _case_row(case_rows, f"{dataset_id}_gnss_outage_true_iekf_best")

    imu_noise = _shared_text(
        (
            f"σ_a = {_format_number(float(eskf.noise.get('sigma_acc', 0.0)))} m/s²/√Hz；"
            f"σ_ω = {_format_number(float(eskf.noise.get('sigma_gyro', 0.0)))} rad/s/√Hz"
        ),
        (
            f"σ_a = {_format_number(float(inekf.noise.get('sigma_acc', 0.0)))} m/s²/√Hz；"
            f"σ_ω = {_format_number(float(inekf.noise.get('sigma_gyro', 0.0)))} rad/s/√Hz"
        ),
    )
    odo_noise = _shared_text(
        f"{_format_number(float(eskf.constraints.get('sigma_odo', 0.0)))} m/s",
        f"{_format_number(float(inekf.constraints.get('sigma_odo', 0.0)))} m/s",
    )
    nhc_noise = _shared_text(
        (
            f"横向 {_format_number(float(eskf.constraints.get('sigma_nhc_y', 0.0)))} m/s；"
            f"垂向 {_format_number(float(eskf.constraints.get('sigma_nhc_z', 0.0)))} m/s"
        ),
        (
            f"横向 {_format_number(float(inekf.constraints.get('sigma_nhc_y', 0.0)))} m/s；"
            f"垂向 {_format_number(float(inekf.constraints.get('sigma_nhc_z', 0.0)))} m/s"
        ),
    )
    zupt_noise = _shared_text(
        f"{_format_number(float(eskf.constraints.get('sigma_zupt', 0.0)))} m/s",
        f"{_format_number(float(inekf.constraints.get('sigma_zupt', 0.0)))} m/s",
    )
    uwb_noise = _shared_text(
        f"{_format_number(float(eskf.noise.get('sigma_uwb', 0.0)))} m",
        f"{_format_number(float(inekf.noise.get('sigma_uwb', 0.0)))} m",
    )

    full_odo = _display_rate(full_row["odo_label"])
    gnss30_eskf_odo = _display_rate(gnss30_eskf_row["odo_label"])
    gnss30_inekf_odo = _display_rate(gnss30_inekf_row["odo_label"])
    outage_eskf_odo = _display_rate(outage_eskf_row["odo_label"])
    outage_inekf_odo = _display_rate(outage_inekf_row["odo_label"])

    full_nhc = _display_rate(full_row["nhc_label"])
    gnss30_eskf_nhc = _display_rate(gnss30_eskf_row["nhc_label"])
    gnss30_inekf_nhc = _display_rate(gnss30_inekf_row["nhc_label"])
    outage_eskf_nhc = _display_rate(outage_eskf_row["nhc_label"])
    outage_inekf_nhc = _display_rate(outage_inekf_row["nhc_label"])

    gnss_pos_scene = "全程GNSS：1 Hz，全程；GNSS30：1 Hz，仅前 30% 时段；周期开断：1 Hz，900 s 收敛后按 300 s on / 120 s off 断续。"

    if dataset_id == "data4":
        gnss_vel_available = "有，1 Hz"
        gnss_vel_noise = "使用 GNSS 文件给出的 σ_vn / σ_ve / σ_vd；若缺失则回退为 0.5 m/s，并设置 1e-4 m/s 下限"
        gnss_vel_scene = "全程GNSS：1 Hz，全程；GNSS30：1 Hz，仅前 30% 时段；周期开断：1 Hz，并随 GNSS on/off 窗口同步断续。"
        gnss_vel_note = "仅 data4 含 GNSS 速度信息，因此位置与速度双链路同时可用。"
    else:
        gnss_vel_available = "无该量测"
        gnss_vel_noise = "无该量测"
        gnss_vel_scene = "全程GNSS / GNSS30 / 周期开断均无该量测。"
        gnss_vel_note = "data2 的 GNSS 文件仅含位置与位置标准差，不含速度与速度标准差。"

    if full_odo == gnss30_eskf_odo == gnss30_inekf_odo == outage_eskf_odo == outage_inekf_odo:
        odo_scene = f"三组场景均为{full_odo}。"
    else:
        odo_scene = (
            f"全程GNSS：{full_odo}；"
            f"GNSS30：{_scene_pair_text(gnss30_eskf_odo, gnss30_inekf_odo)}；"
            f"周期开断：{_scene_pair_text(outage_eskf_odo, outage_inekf_odo)}。"
        )

    if full_nhc == gnss30_eskf_nhc == gnss30_inekf_nhc == outage_eskf_nhc == outage_inekf_nhc:
        nhc_scene = f"三组场景均为{full_nhc}。"
    else:
        nhc_scene = (
            f"全程GNSS：{full_nhc}；"
            f"GNSS30：{_scene_pair_text(gnss30_eskf_nhc, gnss30_inekf_nhc)}；"
            f"周期开断：{_scene_pair_text(outage_eskf_nhc, outage_inekf_nhc)}。"
        )

    rows = [
        {
            "信号源": "IMU 传播",
            "原始数据可用性": "有，约 200 Hz",
            "量测噪声": imu_noise,
            "公共更新节奏": "连续传播，约 200 Hz",
            "场景差异": "三组场景均连续传播。",
            "说明": "IMU 传播贯穿全部场景，是所有误差状态的基础驱动源。",
        },
        {
            "信号源": "GNSS 位置",
            "原始数据可用性": "有，1 Hz",
            "量测噪声": "使用 GNSS 文件给出的 σ_N / σ_E / σ_D，按每轴标准差构造位置量测协方差",
            "公共更新节奏": "1 Hz",
            "场景差异": gnss_pos_scene,
            "说明": "三组实验之间的主要差别首先来自 GNSS 位置调度方式的变化。",
        },
        {
            "信号源": "GNSS 速度",
            "原始数据可用性": gnss_vel_available,
            "量测噪声": gnss_vel_noise,
            "公共更新节奏": "1 Hz" if dataset_id == "data4" else "无该量测",
            "场景差异": gnss_vel_scene,
            "说明": gnss_vel_note,
        },
        {
            "信号源": "ODO",
            "原始数据可用性": "有，约 200 Hz",
            "量测噪声": odo_noise,
            "公共更新节奏": "原始频率，约 200 Hz",
            "场景差异": odo_scene,
            "说明": "本报告各场景均保持 ODO 为原始频率，差别主要集中在 GNSS 调度与 NHC 更新策略。",
        },
        {
            "信号源": "NHC",
            "原始数据可用性": "可由车体运动学实时构造",
            "量测噪声": nhc_noise,
            "公共更新节奏": "可按原始频率触发，约 200 Hz",
            "场景差异": nhc_scene,
            "说明": "GNSS30 与周期开断场景均采用各自当前正式最优 NHC 频率；周期开断直接复用 GNSS30 的最优设置。",
        },
        {
            "信号源": "ZUPT",
            "原始数据可用性": "未使用",
            "量测噪声": zupt_noise,
            "公共更新节奏": "按静止窗口触发（本报告未启用）",
            "场景差异": "三组场景均未启用。",
            "说明": "配置中保留 ZUPT 参数，但当前报告所有正式案例均关闭该量测。",
        },
        {
            "信号源": "UWB",
            "原始数据可用性": "未使用",
            "量测噪声": uwb_noise,
            "公共更新节奏": "按外部数据触发（本报告未启用）",
            "场景差异": "三组场景均未启用。",
            "说明": "本报告聚焦 INS/GNSS/ODO/NHC 组合导航，不包含 UWB 更新。",
        },
    ]
    return pd.DataFrame(rows, columns=OBS_TABLE_COLUMNS)


def _intro_sections(case_rows: dict[str, dict[str, Any]]) -> tuple[IntroSectionSpec, ...]:
    data2_eskf = _load_filter_profile(_case_row(case_rows, "data2_gnss30_eskf_best"))
    data2_inekf = _load_filter_profile(_case_row(case_rows, "data2_gnss30_true_iekf_best"))
    data4_eskf = _load_filter_profile(_case_row(case_rows, "data4_gnss30_eskf_best"))
    data4_inekf = _load_filter_profile(_case_row(case_rows, "data4_gnss30_true_iekf_best"))

    dataset_diff_df = pd.DataFrame(
        [
            {
                "数据集": "data2",
                "GNSS 位置信息": "有",
                "GNSS 速度信息": "无",
                "速度水平": "更高",
                "转弯丰富度": "更多",
                "运动状态复杂度": "更复杂",
                "备注": "GNSS 关闭后的主工况约 1691.8 s / 20.21 km / 11.95 m/s，更容易暴露长时航向与外参耦合问题。",
            },
            {
                "数据集": "data4",
                "GNSS 位置信息": "有",
                "GNSS 速度信息": "有",
                "速度水平": "更低",
                "转弯丰富度": "更少",
                "运动状态复杂度": "更简单",
                "备注": "GNSS 关闭后的主工况约 1421.0 s / 6.66 km / 4.69 m/s，速度更低、机动更平缓。",
            },
        ]
    )
    experiment_design_df = pd.DataFrame(
        [
            {
                "实验组": "全程 GNSS vs 前 30% GNSS",
                "对照关系": "同一数据集内，仅比较 GNSS 调度方式变化带来的退化量级；对照算法固定为 ESKF。",
                "GNSS 调度": "一组全程保留 GNSS；另一组仅在前 30% 时段保留 GNSS，后续依赖惯导与约束。",
                "说明": "全程 GNSS 组保持原始 NHC；GNSS30 组采用当前正式 ESKF 最优 NHC。状态图按同数据集内同案例只首展示一次去重。",
            },
            {
                "实验组": "GNSS30 下 ESKF vs InEKF",
                "对照关系": "在完全相同的 GNSS30 工况下，对比两种滤波结构的表现差异。",
                "GNSS 调度": "两组都仅在前 30% 时段使用 GNSS，后续依赖惯导与约束。",
                "说明": "ESKF 与 InEKF 各自采用当前正式最优 NHC；若案例已在前页展示状态图，本页只保留回链提示。",
            },
            {
                "实验组": "GNSS 周期开断",
                "对照关系": "对照组为全程 GNSS ESKF；实验组为周期断续 GNSS 下的 ESKF 与 InEKF。",
                "GNSS 调度": "先 900 s 全 GNSS 收敛，随后按 300 s on / 120 s off 周期断续，直至数据结束。",
                "说明": "周期开断场景直接复用同算法在 GNSS30 下的正式最优 NHC；状态图继续按同数据集内首展示去重。",
            },
        ]
    )

    return (
        IntroSectionSpec(
            title="数据集差异",
            body_html=(
                '<p class="section-copy">'
                "本节强调的是两组数据在传感器构成与运动状态上的客观差异，而不是由实验结果反推的难度判断。"
                "</p>"
            ),
            tables=(ReaderTableSpec(title="data2 / data4 对照", dataframe=dataset_diff_df, table_kind="dataset-diff"),),
        ),
        IntroSectionSpec(
            title="实验设计",
            body_html=(
                '<p class="section-copy">'
                "报告中的 InEKF 统一指当前正式 InEKF 方案。以下三组对照共用同一批正式结果，只重组展示视角，不重新定义实验口径。"
                "</p>"
            ),
            tables=(ReaderTableSpec(title="三组实验的对照关系与调度方式", dataframe=experiment_design_df, table_kind="experiment-design"),),
        ),
        IntroSectionSpec(
            title="data2 关键参数配置",
            body_html=_dataset_parameter_intro("data2", data2_eskf, data2_inekf),
            tables=(
                ReaderTableSpec(
                    title="状态与噪声设计表",
                    dataframe=_state_noise_table("data2", data2_eskf, data2_inekf),
                    table_kind="state-config",
                ),
                ReaderTableSpec(
                    title="观测源与更新设计表",
                    dataframe=_observation_design_table("data2", case_rows, data2_eskf, data2_inekf),
                    note_html=(
                        '<p class="table-note">'
                        "data2 的 GNSS30 正式最优 NHC 为 ESKF 30 Hz、InEKF 0.75 Hz；周期开断场景直接复用这一设置。"
                        "</p>"
                    ),
                    table_kind="observation-config",
                ),
            ),
        ),
        IntroSectionSpec(
            title="data4 关键参数配置",
            body_html=_dataset_parameter_intro("data4", data4_eskf, data4_inekf),
            tables=(
                ReaderTableSpec(
                    title="状态与噪声设计表",
                    dataframe=_state_noise_table("data4", data4_eskf, data4_inekf),
                    table_kind="state-config",
                ),
                ReaderTableSpec(
                    title="观测源与更新设计表",
                    dataframe=_observation_design_table("data4", case_rows, data4_eskf, data4_inekf),
                    note_html=(
                        '<p class="table-note">'
                        "data4 的 GNSS30 正式最优 NHC 在 ESKF 与 InEKF 下均为原始频率；周期开断场景直接复用这一设置。"
                        "</p>"
                    ),
                    table_kind="observation-config",
                ),
            ),
        ),
    )


def _growth_text(candidate: float, reference: float) -> str:
    if not math.isfinite(candidate) or not math.isfinite(reference) or reference <= 0.0:
        return "nan"
    ratio = candidate / reference
    if ratio >= 1.0:
        return f"约增至 {ratio:.1f} 倍"
    return f"约降至 {ratio:.2f} 倍"


def _reduction_text(candidate: float, reference: float) -> str:
    if not math.isfinite(candidate) or not math.isfinite(reference) or reference <= 0.0:
        return "nan"
    delta = (1.0 - candidate / reference) * 100.0
    return f"{delta:.1f}%"


def _section_analysis(dataset_id: str, family_id: str, rows: dict[str, dict[str, Any]]) -> str:
    if family_id == "full_vs_gnss30":
        full_row = _case_row(rows, f"{dataset_id}_full_gnss_eskf")
        gnss30_row = _case_row(rows, f"{dataset_id}_gnss30_eskf_best")
        items = [
            f"同一 ESKF 下，全程 GNSS 的三维 RMSE 为 <strong>{_fmt(_float(full_row, 'rmse_3d_m'))} m</strong>；仅保留前 30% GNSS 后，GNSS30 结果为 <strong>{_fmt(_float(gnss30_row, 'rmse_3d_m'))} m</strong>。",
            f"该 GNSS30 场景下的正式 NHC 设置为 <strong>{_display_rate(gnss30_row['nhc_label'])}</strong>；相对全程 GNSS，误差量级 <strong>{_growth_text(_float(gnss30_row, 'rmse_3d_m'), _float(full_row, 'rmse_3d_m'))}</strong>。",
            "这一页只用于量化 GNSS 调度变化本身带来的退化量级，便于把“调度影响”与后续的“滤波结构差异”分开观察。",
        ]
        return _analysis_card("结果分析", items)

    if family_id == "gnss30_compare":
        eskf_row = _case_row(rows, f"{dataset_id}_gnss30_eskf_best")
        inekf_row = _case_row(rows, f"{dataset_id}_gnss30_true_iekf_best")
        eskf_rmse = _float(eskf_row, "rmse_3d_m")
        inekf_rmse = _float(inekf_row, "rmse_3d_m")
        better = "InEKF" if inekf_rmse < eskf_rmse else "ESKF"
        items = [
            f"在同一 GNSS30 工况下，ESKF 使用 <strong>{_display_rate(eskf_row['nhc_label'])}</strong>，三维 RMSE 为 <strong>{_fmt(eskf_rmse)} m</strong>；InEKF 使用 <strong>{_display_rate(inekf_row['nhc_label'])}</strong>，三维 RMSE 为 <strong>{_fmt(inekf_rmse)} m</strong>。",
            f"按当前正式结果，表现更优的是 <strong>{better}</strong>；若以 ESKF 为参考，InEKF 的三维 RMSE 变化为 <strong>{_reduction_text(inekf_rmse, eskf_rmse)}</strong>。",
            "这一页只比较“同一 GNSS30 调度下的 ESKF 与 InEKF”，并且两者都采用各自当前正式最优 NHC 设置。",
        ]
        return _analysis_card("结果分析", items)

    if family_id == "outage_cycle":
        full_row = _case_row(rows, f"{dataset_id}_full_gnss_eskf")
        eskf_row = _case_row(rows, f"{dataset_id}_gnss_outage_eskf_best")
        inekf_row = _case_row(rows, f"{dataset_id}_gnss_outage_true_iekf_best")
        items = [
            f"控制组 <strong>全程GNSS ESKF</strong> 的三维 RMSE 为 <strong>{_fmt(_float(full_row, 'rmse_3d_m'))} m</strong>；在 900 s 收敛后按 300 s on / 120 s off 断续 GNSS 时，ESKF 为 <strong>{_fmt(_float(eskf_row, 'rmse_3d_m'))} m</strong>，InEKF 为 <strong>{_fmt(_float(inekf_row, 'rmse_3d_m'))} m</strong>。",
            f"两条周期开断曲线都直接复用了 GNSS30 场景下的正式最优 NHC：ESKF 为 <strong>{_display_rate(eskf_row['nhc_label'])}</strong>，InEKF 为 <strong>{_display_rate(inekf_row['nhc_label'])}</strong>。",
            "这一页对应的是“同一断续模板下，ESKF 与 InEKF 的稳态保持能力”对照，并以全程 GNSS ESKF 作为控制组提供参考上界。",
        ]
        return _analysis_card("结果分析", items)

    return ""


def _section_specs(case_rows: dict[str, dict[str, Any]]) -> tuple[DatasetSectionSpec, ...]:
    specs: list[DatasetSectionSpec] = []
    for dataset_id in ("data2", "data4"):
        full_row = _case_row(case_rows, f"{dataset_id}_full_gnss_eskf")
        gnss30_eskf_row = _case_row(case_rows, f"{dataset_id}_gnss30_eskf_best")
        gnss30_inekf_row = _case_row(case_rows, f"{dataset_id}_gnss30_true_iekf_best")
        outage_eskf_row = _case_row(case_rows, f"{dataset_id}_gnss_outage_eskf_best")
        outage_inekf_row = _case_row(case_rows, f"{dataset_id}_gnss_outage_true_iekf_best")

        specs.extend(
            [
                DatasetSectionSpec(
                    exp_id=f"{dataset_id}-full-vs-gnss30",
                    title="全程 GNSS vs 前30% GNSS",
                    subtitle="同一 ESKF 下，仅比较 GNSS 调度方式变化带来的性能退化量级。",
                    dataset_id=dataset_id,
                    family_id="full_vs_gnss30",
                    case_specs=(
                        _make_case_spec(full_row, CASE_COLORS["full"]),
                        _make_case_spec(gnss30_eskf_row, CASE_COLORS["eskf_best"]),
                    ),
                    state_galleries=(
                        _make_gallery_spec(full_row),
                        _make_gallery_spec(gnss30_eskf_row),
                    ),
                ),
                DatasetSectionSpec(
                    exp_id=f"{dataset_id}-gnss30-eskf-vs-inekf",
                    title="GNSS30 下 ESKF vs InEKF",
                    subtitle="同一 GNSS30 工况下，ESKF 与 InEKF 均采用各自当前正式最优 NHC。",
                    dataset_id=dataset_id,
                    family_id="gnss30_compare",
                    case_specs=(
                        _make_case_spec(gnss30_eskf_row, CASE_COLORS["eskf_best"]),
                        _make_case_spec(gnss30_inekf_row, CASE_COLORS["inekf_best"]),
                    ),
                    state_galleries=(
                        _make_gallery_spec(gnss30_eskf_row),
                        _make_gallery_spec(gnss30_inekf_row),
                    ),
                ),
                DatasetSectionSpec(
                    exp_id=f"{dataset_id}-gnss-outage-cycle",
                    title="GNSS 周期开断",
                    subtitle="对照组为全程 GNSS ESKF；断续组采用 900 s 收敛后 300 s on / 120 s off 模板。",
                    dataset_id=dataset_id,
                    family_id="outage_cycle",
                    case_specs=(
                        _make_case_spec(full_row, CASE_COLORS["full"]),
                        _make_case_spec(outage_eskf_row, CASE_COLORS["outage_eskf"]),
                        _make_case_spec(outage_inekf_row, CASE_COLORS["outage_inekf"]),
                    ),
                    state_galleries=(
                        _make_gallery_spec(full_row),
                        _make_gallery_spec(outage_eskf_row),
                        _make_gallery_spec(outage_inekf_row),
                    ),
                ),
            ]
        )
    return tuple(specs)


def _groups() -> tuple[DatasetGroupSpec, ...]:
    return (
        DatasetGroupSpec(group_id="intro", title="实验说明"),
        DatasetGroupSpec(group_id="data2", title="data2"),
        DatasetGroupSpec(group_id="data4", title="data4"),
    )


def build_dataset_partitioned_report(target_points: int = 4000) -> DatasetPartitionedReport:
    manifest_path, manifest = _load_manifest()
    case_rows = _case_map(manifest)
    groups = _groups()
    section_specs = _section_specs(case_rows)
    section_results = build_report_from_sections(
        [
            SectionSpec(
                exp_id=spec.exp_id,
                title=spec.title,
                subtitle=spec.subtitle,
                cases=spec.case_specs,
                shade_gnss_windows=(spec.family_id == "outage_cycle"),
            )
            for spec in section_specs
        ],
        target_points=target_points,
    )
    result_map = {section.spec.exp_id: section for section in section_results}

    pages: list[DatasetReportPage] = [
        DatasetReportPage(
            page_id="page-intro",
            group_id="intro",
            nav_label="实验说明",
            title=REPORT_TITLE,
            subtitle="实验说明",
            intro_sections=_intro_sections(case_rows),
        )
    ]

    for dataset_id in ("data2", "data4"):
        dataset_specs = [spec for spec in section_specs if spec.dataset_id == dataset_id]
        for idx, spec in enumerate(dataset_specs, start=1):
            pages.append(
                DatasetReportPage(
                    page_id=f"page-{dataset_id}-{idx}",
                    group_id=dataset_id,
                    nav_label=spec.title,
                    title=spec.title,
                    subtitle=spec.subtitle,
                    section_result=result_map[spec.exp_id],
                    analysis_html=_section_analysis(dataset_id, spec.family_id, case_rows),
                    state_galleries=spec.state_galleries,
                )
            )

    return DatasetPartitionedReport(
        report_title=REPORT_TITLE,
        groups=groups,
        pages=pages,
        manifest_path=manifest_path,
    )
