from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import yaml

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_E2 = 1.0 - (WGS84_B / WGS84_A) ** 2

REPO_ROOT = Path(__file__).resolve().parents[2]

NEW_FORMAT_COLS_28 = [
    "timestamp",
    "fused_x",
    "fused_y",
    "fused_z",
    "fused_vx",
    "fused_vy",
    "fused_vz",
    "fused_roll",
    "fused_pitch",
    "fused_yaw",
    "mounting_pitch",
    "mounting_yaw",
    "odo_scale",
    "sg_x",
    "sg_y",
    "sg_z",
    "sa_x",
    "sa_y",
    "sa_z",
    "ba_x",
    "ba_y",
    "ba_z",
    "bg_x",
    "bg_y",
    "bg_z",
    "lever_x",
    "lever_y",
    "lever_z",
]

NEW_FORMAT_COLS_31 = NEW_FORMAT_COLS_28 + [
    "gnss_lever_x",
    "gnss_lever_y",
    "gnss_lever_z",
]

SECTION_COLORS = {
    "group1": ["#2E75D6", "#1FA187"],
    "group2": ["#2E75D6", "#1696A3"],
    "group3": ["#2E75D6", "#E67E22"],
    "group4": ["#C0392B", "#27AE60"],
    "group5": ["#C0392B", "#27AE60"],
    "group6": ["#2E75D6", "#E67E22"],
    "group7": ["#2E75D6", "#1FA187", "#E67E22", "#8D77C6"],
    "group8": ["#2E75D6", "#1696A3", "#D39B3A", "#B55D89"],
    "group9": ["#2E75D6", "#1FA187", "#E67E22", "#B55D89"],
    "group10": ["#C0392B", "#27AE60", "#D39B3A", "#8D77C6"],
    "group11": ["#2E75D6", "#C0392B"],
}

REPORT_FONT_SANS = '"Times New Roman", "Songti SC", "SimSun", "STSong", serif'
REPORT_FONT_SERIF = '"Times New Roman", "Songti SC", "SimSun", "STSong", serif'

CATEGORY_SPECS = [
    (
        "n_velocity",
        "n系速度误差",
        [
            ("vn_err_mps", "北向速度误差 <i>v</i><sub>n</sub> [m/s]"),
            ("ve_err_mps", "东向速度误差 <i>v</i><sub>e</sub> [m/s]"),
            ("vd_err_mps", "地向速度误差 <i>v</i><sub>d</sub> [m/s]"),
        ],
    ),
    (
        "v_velocity",
        "v系速度误差",
        [
            ("vv_x_err_mps", "前向速度误差 <i>v</i><sub>x</sub><sup>v</sup> [m/s]"),
            ("vv_y_err_mps", "横向速度误差 <i>v</i><sub>y</sub><sup>v</sup> [m/s]"),
            ("vv_z_err_mps", "垂向速度误差 <i>v</i><sub>z</sub><sup>v</sup> [m/s]"),
        ],
    ),
    (
        "vehicle_heading",
        "v系航向误差",
        [("vehicle_heading_err_deg", "v系航向误差 [deg]")],
    ),
    (
        "level_angles",
        "体系姿态角误差",
        [
            ("pitch_err_deg", "俯仰角误差 [deg]"),
            ("roll_err_deg", "横滚角误差 [deg]"),
        ],
    ),
    (
        "altitude",
        "高程误差",
        [("alt_err_m", "高程误差 [m]")],
    ),
]


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    sol_path: str
    config_path: str
    color: str


@dataclass(frozen=True)
class SectionSpec:
    exp_id: str
    title: str
    subtitle: str
    cases: tuple[CaseSpec, ...]
    shade_gnss_windows: bool = False


@dataclass
class TruthBundle:
    time: np.ndarray
    time_sec: np.ndarray
    lat_deg: np.ndarray
    lon_deg: np.ndarray
    alt_m: np.ndarray
    vn_mps: np.ndarray
    ve_mps: np.ndarray
    vd_mps: np.ndarray
    roll_deg: np.ndarray
    pitch_deg: np.ndarray
    yaw_deg: np.ndarray
    ecef_xyz_m: np.ndarray
    origin_xyz_m: np.ndarray
    origin_lat_deg: float
    origin_lon_deg: float
    trajectory_plot_df: pd.DataFrame


@dataclass
class CaseResult:
    spec: CaseSpec
    overview: dict[str, Any]
    plot_df: pd.DataFrame
    stats_by_category: dict[str, dict[str, dict[str, float]]]
    gnss_windows_s: list[tuple[float, float]]


@dataclass
class SectionResult:
    spec: SectionSpec
    truth_plot_df: pd.DataFrame
    cases: list[CaseResult]
    overview_table: pd.DataFrame
    stats_tables: dict[str, pd.DataFrame]
    split_time_s: float | None
    gnss_windows_s: list[tuple[float, float]]
    trajectory_fig: go.Figure
    figures: dict[str, go.Figure]


REPORT_SECTIONS: tuple[SectionSpec, ...] = (
    SectionSpec(
        exp_id="EXP-20260309-interactive-report-group1",
        title="第一组：全程GNSS 与前30%GNSS",
        subtitle="data4 全程 GNSS ESKF 与 data4 前 30% GNSS ESKF 对比",
        cases=(
            CaseSpec(
                case_id="data4_baseline_eskf",
                label="data4 基线 ESKF",
                sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_baseline_eskf_doc.txt",
                config_path="output/review/EXP-20260305-data4-main4-regression-r1/cfg_baseline_eskf.yaml",
                color=SECTION_COLORS["group1"][0],
            ),
            CaseSpec(
                case_id="data4_gnss30_eskf",
                label="data4 GNSS30 ESKF",
                sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_gnss30_eskf_doc.txt",
                config_path="output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml",
                color=SECTION_COLORS["group1"][1],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260309-interactive-report-group2",
        title="第二组：ESKF 与 InEKF",
        subtitle="data4 前 30% GNSS 条件下 ESKF 与 InEKF 对比",
        cases=(
            CaseSpec(
                case_id="data4_gnss30_eskf",
                label="data4 GNSS30 ESKF",
                sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_gnss30_eskf_doc.txt",
                config_path="output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml",
                color=SECTION_COLORS["group2"][0],
            ),
            CaseSpec(
                case_id="data4_gnss30_inekf",
                label="data4 GNSS30 InEKF",
                sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_gnss30_true_iekf_doc.txt",
                config_path="config_data4_gnss30_true_iekf.yaml",
                color=SECTION_COLORS["group2"][1],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260310-data2-gnss-outage-cycle-r1",
        title="第三组：GNSS 周期开断",
        subtitle="data2 ESKF：全程 GNSS 与 900 s 收敛后 300 s/120 s 周期断续 GNSS",
        cases=(
            CaseSpec(
                case_id="data2_baseline_eskf",
                label="data2 基线 ESKF",
                sol_path="SOL_data2_baseline_eskf.txt",
                config_path="config_data2_baseline_eskf.yaml",
                color=SECTION_COLORS["group3"][0],
            ),
            CaseSpec(
                case_id="data2_outage_eskf_cycle",
                label="data2 ESKF 周期开断",
                sol_path="output/review/EXP-20260310-data2-gnss-outage-cycle-r1/SOL_data2_eskf_gnss_outage_cycle.txt",
                config_path="output/review/EXP-20260310-data2-gnss-outage-cycle-r1/cfg_data2_eskf_gnss_outage_cycle.yaml",
                color=SECTION_COLORS["group3"][1],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260310-gnss-lever-fix-eskf",
        title="第四组：GNSS 杆臂偏差对照（ESKF）",
        subtitle="data4 全程 GNSS，GNSS 杆臂自由估计与固定真值对比（ESKF）",
        cases=(
            CaseSpec(
                case_id="data4_lever_eskf_free",
                label="ESKF 自由杆臂",
                sol_path="output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_eskf_free.txt",
                config_path="output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_free.yaml",
                color=SECTION_COLORS["group4"][0],
            ),
            CaseSpec(
                case_id="data4_lever_eskf_fixed",
                label="ESKF 固定杆臂",
                sol_path="output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_eskf_fixed.txt",
                config_path="output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_fixed.yaml",
                color=SECTION_COLORS["group4"][1],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260310-gnss-lever-fix-true-iekf",
        title="第五组：GNSS杆臂偏差对照（InEKF）",
        subtitle="data4 全程 GNSS，GNSS 杆臂自由估计与固定真值对比（InEKF）",
        cases=(
            CaseSpec(
                case_id="data4_lever_inekf_free",
                label="InEKF 自由杆臂",
                sol_path="output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_true_iekf_free.txt",
                config_path="output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_free.yaml",
                color=SECTION_COLORS["group5"][0],
            ),
            CaseSpec(
                case_id="data4_lever_inekf_fixed",
                label="InEKF 固定杆臂",
                sol_path="output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_true_iekf_fixed.txt",
                config_path="output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_fixed.yaml",
                color=SECTION_COLORS["group5"][1],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260310-gnss-vel-effect-r1",
        title="第六组：GNSS速度有无对照组（InEKF）",
        subtitle="data4 全程 GNSS，InEKF，含 GNSS 速度与仅位置约束对比",
        cases=(
            CaseSpec(
                case_id="data4_vel_with",
                label="InEKF 含速度",
                sol_path="output/review/EXP-20260310-gnss-vel-effect-r1/SOL_data4_vel_with.txt",
                config_path="output/review/EXP-20260310-gnss-vel-effect-r1/cfg_with_vel.yaml",
                color=SECTION_COLORS["group6"][0],
            ),
            CaseSpec(
                case_id="data4_vel_no",
                label="InEKF 无速度",
                sol_path="output/review/EXP-20260310-gnss-vel-effect-r1/SOL_data4_vel_no.txt",
                config_path="output/review/EXP-20260310-gnss-vel-effect-r1/cfg_no_vel.yaml",
                color=SECTION_COLORS["group6"][1],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260310-inekf-mechanism-group7",
        title="第七组：InEKF 作用机理（data4 GNSS30）",
        subtitle="data4 前 30% GNSS：ESKF、InEKF 与 post-GNSS 冻结（bg/mounting）对比",
        cases=(
            CaseSpec(
                case_id="data4_gnss30_eskf_ref",
                label="data4 GNSS30 ESKF",
                sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_gnss30_eskf_doc.txt",
                config_path="output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml",
                color=SECTION_COLORS["group7"][0],
            ),
            CaseSpec(
                case_id="data4_gnss30_true_full",
                label="data4 InEKF 完整估计",
                sol_path="output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_full.txt",
                config_path="output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_full.yaml",
                color=SECTION_COLORS["group7"][1],
            ),
            CaseSpec(
                case_id="data4_gnss30_true_freeze_bg",
                label="data4 冻结 bg",
                sol_path="output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_freeze_bg.txt",
                config_path="output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_freeze_bg.yaml",
                color=SECTION_COLORS["group7"][2],
            ),
            CaseSpec(
                case_id="data4_gnss30_true_freeze_mount",
                label="data4 冻结 mounting",
                sol_path="output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_freeze_mount.txt",
                config_path="output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_freeze_mount.yaml",
                color=SECTION_COLORS["group7"][3],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260310-inekf-mechanism-group8",
        title="第八组：InEKF 作用机理（data2 GNSS30）",
        subtitle="data2 前 30% GNSS：ESKF、InEKF 与 post-GNSS 冻结（bg/mounting）对比",
        cases=(
            CaseSpec(
                case_id="data2_gnss30_eskf_ref",
                label="data2 GNSS30 ESKF 参考",
                sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data2_gnss30_eskf_ref.txt",
                config_path="output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml",
                color=SECTION_COLORS["group8"][0],
            ),
            CaseSpec(
                case_id="data2_gnss30_true_full",
                label="data2 InEKF 完整估计",
                sol_path="output/review/20260306-phase2c-bg-freeze/SOL_full.txt",
                config_path="output/review/20260306-phase2c-bg-freeze/cfg_full.yaml",
                color=SECTION_COLORS["group8"][1],
            ),
            CaseSpec(
                case_id="data2_gnss30_true_freeze_bg",
                label="data2 冻结 bg",
                sol_path="output/review/20260306-phase2c-bg-freeze/SOL_freeze_bg.txt",
                config_path="output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg.yaml",
                color=SECTION_COLORS["group8"][2],
            ),
            CaseSpec(
                case_id="data2_gnss30_true_freeze_mount",
                label="data2 冻结 mounting",
                sol_path="output/review/20260306-phase2c-bg-freeze/SOL_freeze_mount.txt",
                config_path="output/review/20260306-phase2c-bg-freeze/cfg_freeze_mount.yaml",
                color=SECTION_COLORS["group8"][3],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260311-interactive-report-group9",
        title="第九组：NHC频率扫描（data2 GNSS30 ESKF）",
        subtitle="固定 ODO=raw，对比 raw、50 Hz、30 Hz 与 1 Hz 的 NHC 更新频率",
        cases=(
            CaseSpec(
                case_id="data2_gnss30_eskf_nhc_raw",
                label="raw/raw",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/data2_gnss30_eskf/SOL_matched_odo_raw_nhc_raw.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/data2_gnss30_eskf/cfg_matched_odo_raw_nhc_raw.yaml",
                color=SECTION_COLORS["group9"][0],
            ),
            CaseSpec(
                case_id="data2_gnss30_eskf_nhc_50hz",
                label="NHC 50 Hz",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/data2_gnss30_eskf/SOL_fixed_odo_nhc_sweep_odo_raw_nhc_50hz.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/data2_gnss30_eskf/cfg_fixed_odo_nhc_sweep_odo_raw_nhc_50hz.yaml",
                color=SECTION_COLORS["group9"][1],
            ),
            CaseSpec(
                case_id="data2_gnss30_eskf_nhc_30hz",
                label="NHC 30 Hz",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2b-data2-gnss30-eskf-refine/data2_gnss30_eskf/SOL_fixed_odo_nhc_sweep_odo_raw_nhc_30.000hz.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2b-data2-gnss30-eskf-refine/data2_gnss30_eskf/cfg_fixed_odo_nhc_sweep_odo_raw_nhc_30.000hz.yaml",
                color=SECTION_COLORS["group9"][2],
            ),
            CaseSpec(
                case_id="data2_gnss30_eskf_nhc_1hz",
                label="NHC 1 Hz",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/data2_gnss30_eskf/SOL_fixed_odo_nhc_sweep_odo_raw_nhc_1hz.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/data2_gnss30_eskf/cfg_fixed_odo_nhc_sweep_odo_raw_nhc_1hz.yaml",
                color=SECTION_COLORS["group9"][3],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260311-interactive-report-group10",
        title="第十组：NHC频率扫描（data2 GNSS30 InEKF）",
        subtitle="固定 ODO=raw，对比 raw、10 Hz、1 Hz 与 0.75 Hz 的 NHC 更新频率（true_iekf 实现）",
        cases=(
            CaseSpec(
                case_id="data2_gnss30_true_iekf_nhc_raw",
                label="raw/raw",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf/data2_gnss30_true_iekf/SOL_matched_odo_raw_nhc_raw.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf/data2_gnss30_true_iekf/cfg_matched_odo_raw_nhc_raw.yaml",
                color=SECTION_COLORS["group10"][0],
            ),
            CaseSpec(
                case_id="data2_gnss30_true_iekf_nhc_10hz",
                label="NHC 10 Hz",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf/data2_gnss30_true_iekf/SOL_fixed_odo_nhc_sweep_odo_raw_nhc_10hz.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf/data2_gnss30_true_iekf/cfg_fixed_odo_nhc_sweep_odo_raw_nhc_10hz.yaml",
                color=SECTION_COLORS["group10"][1],
            ),
            CaseSpec(
                case_id="data2_gnss30_true_iekf_nhc_1hz",
                label="NHC 1 Hz",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf/data2_gnss30_true_iekf/SOL_fixed_odo_nhc_sweep_odo_raw_nhc_1hz.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf/data2_gnss30_true_iekf/cfg_fixed_odo_nhc_sweep_odo_raw_nhc_1hz.yaml",
                color=SECTION_COLORS["group10"][2],
            ),
            CaseSpec(
                case_id="data2_gnss30_true_iekf_nhc_075hz",
                label="NHC 0.75 Hz",
                sol_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3b-data2-gnss30-true-iekf-refine/data2_gnss30_true_iekf/SOL_fixed_odo_nhc_sweep_odo_raw_nhc_0.750hz.txt",
                config_path="output/review/EXP-20260311-odo-nhc-rate-sweep-r3b-data2-gnss30-true-iekf-refine/data2_gnss30_true_iekf/cfg_fixed_odo_nhc_sweep_odo_raw_nhc_0.750hz.yaml",
                color=SECTION_COLORS["group10"][3],
            ),
        ),
    ),
    SectionSpec(
        exp_id="EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1",
        title="第十一组：GNSS周期开断（data2 true_iekf最优）",
        subtitle="对照组保持 data2 全GNSS ESKF；实验组使用当前 tuned true_iekf 最优配置（ODO raw + NHC 0.75Hz）并复用同一份 900 s 收敛 + 300/120 s 周期断续 GNSS 文件",
        cases=(
            CaseSpec(
                case_id="data2_baseline_eskf_fresh",
                label="data2 全GNSS ESKF",
                sol_path="output/review/EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1/SOL_data2_baseline_eskf_fresh.txt",
                config_path="output/review/EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1/cfg_data2_baseline_eskf_fresh.yaml",
                color=SECTION_COLORS["group11"][0],
            ),
            CaseSpec(
                case_id="data2_true_tuned_gnss_outage_cycle",
                label="data2 true_iekf最优(0.75Hz) + 周期开断",
                sol_path="output/review/EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1/SOL_data2_true_tuned_gnss_outage_cycle.txt",
                config_path="output/review/EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1/cfg_data2_true_tuned_gnss_outage_cycle.yaml",
                color=SECTION_COLORS["group11"][1],
            ),
        ),
    ),
)


def repo_path(rel_path: str) -> Path:
    return (REPO_ROOT / rel_path).resolve()


def llh_deg_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, h_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * cos_lon
    y = (n + h_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + h_m) * sin_lat
    return np.column_stack((x, y, z))


def ecef_to_lla(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lon = np.arctan2(y, x)
    p = np.sqrt(x * x + y * y)
    lat = np.arctan2(z, p * (1.0 - WGS84_E2))
    for _ in range(5):
        sin_lat = np.sin(lat)
        n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat = np.arctan2(z + WGS84_E2 * n * sin_lat, p)
    sin_lat = np.sin(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / np.cos(lat) - n
    return np.rad2deg(lat), np.rad2deg(lon), alt


def ecef_vel_to_ned(
    vx: np.ndarray, vy: np.ndarray, vz: np.ndarray, lat_deg: np.ndarray, lon_deg: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    vn = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz
    ve = -sin_lon * vx + cos_lon * vy
    vd = -cos_lat * cos_lon * vx - cos_lat * sin_lon * vy - sin_lat * vz
    return vn, ve, vd


def ecef_to_enu(
    xyz_ecef: np.ndarray, origin_xyz: np.ndarray, origin_lat_deg: float, origin_lon_deg: float
) -> np.ndarray:
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)
    rot = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )
    delta = (xyz_ecef - origin_xyz).T
    return (rot @ delta).T


def euler_to_rotation(roll_rad: np.ndarray, pitch_rad: np.ndarray, yaw_rad: np.ndarray) -> np.ndarray:
    roll_rad = np.asarray(roll_rad)
    pitch_rad = np.asarray(pitch_rad)
    yaw_rad = np.asarray(yaw_rad)
    sr = np.sin(roll_rad)
    cr = np.cos(roll_rad)
    sp = np.sin(pitch_rad)
    cp = np.cos(pitch_rad)
    sy = np.sin(yaw_rad)
    cy = np.cos(yaw_rad)
    r11 = cp * cy
    r12 = sr * sp * cy - cr * sy
    r13 = cr * sp * cy + sr * sy
    r21 = cp * sy
    r22 = sr * sp * sy + cr * cy
    r23 = cr * sp * sy - sr * cy
    r31 = -sp
    r32 = sr * cp
    r33 = cr * cp
    return np.stack(
        [
            np.stack([r11, r12, r13], axis=-1),
            np.stack([r21, r22, r23], axis=-1),
            np.stack([r31, r32, r33], axis=-1),
        ],
        axis=-2,
    )


def rotation_to_euler_deg(rot: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pitch = -np.arcsin(np.clip(rot[..., 2, 0], -1.0, 1.0))
    roll = np.arctan2(rot[..., 2, 1], rot[..., 2, 2])
    yaw = np.arctan2(rot[..., 1, 0], rot[..., 0, 0])
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def wrap_angle_deg(angle_deg: np.ndarray) -> np.ndarray:
    return (np.asarray(angle_deg) + 180.0) % 360.0 - 180.0


def relative_euler_error_deg(
    est_rot: np.ndarray, ref_rot: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # Use relative rotation, not direct Euler subtraction, to avoid yaw/roll leakage into pitch.
    rel_rot = np.einsum("...ij,...jk->...ik", est_rot, np.swapaxes(ref_rot, -1, -2))
    roll_err_deg, pitch_err_deg, yaw_err_deg = rotation_to_euler_deg(rel_rot)
    return (
        wrap_angle_deg(roll_err_deg),
        wrap_angle_deg(pitch_err_deg),
        wrap_angle_deg(yaw_err_deg),
    )


def interp_linear(sol_t: np.ndarray, truth_t: np.ndarray, values: np.ndarray) -> np.ndarray:
    return np.interp(sol_t, truth_t, values)


def interp_angle_deg(sol_t: np.ndarray, truth_t: np.ndarray, values_deg: np.ndarray) -> np.ndarray:
    unwrapped = np.unwrap(np.deg2rad(values_deg))
    interp = np.interp(sol_t, truth_t, unwrapped)
    return np.rad2deg(interp)


def decimate_df(df: pd.DataFrame, target_points: int) -> pd.DataFrame:
    if len(df) <= target_points:
        return df.copy()
    idx = np.linspace(0, len(df) - 1, target_points, dtype=int)
    return df.iloc[idx].reset_index(drop=True)


def detect_has_header(path: Path) -> bool:
    with path.open("r", encoding="utf-8") as f:
        first = f.readline().strip()
    if not first:
        return False
    token = first.split()[0]
    return not token.lstrip("-").replace(".", "", 1).isdigit()


def load_solution_frame(path: Path) -> pd.DataFrame:
    has_header = detect_has_header(path)
    df = pd.read_csv(path, sep=r"\s+", header=0 if has_header else None, comment="#")
    if not has_header:
        if df.shape[1] == len(NEW_FORMAT_COLS_31):
            df.columns = NEW_FORMAT_COLS_31
        elif df.shape[1] == len(NEW_FORMAT_COLS_28):
            df.columns = NEW_FORMAT_COLS_28
    required = {
        "timestamp",
        "fused_x",
        "fused_y",
        "fused_z",
        "fused_vx",
        "fused_vy",
        "fused_vz",
        "fused_roll",
        "fused_pitch",
        "fused_yaw",
        "mounting_pitch",
        "mounting_yaw",
    }
    missing = required.difference(df.columns)
    if missing:
        raise RuntimeError(f"solution file missing columns {sorted(missing)}: {path}")
    return df


def load_truth_frame(path: Path) -> pd.DataFrame:
    has_header = detect_has_header(path)
    if has_header:
        df = pd.read_csv(path, sep=r"\s+")
        df = df.rename(columns={"h": "alt"})
    else:
        df = pd.read_csv(path, sep=r"\s+", header=None)
        if df.shape[1] == 10:
            df.columns = ["timestamp", "lat", "lon", "alt", "vn", "ve", "vd", "roll", "pitch", "yaw"]
        else:
            raise RuntimeError(f"unsupported truth file layout: {path} ({df.shape[1]} columns)")
    required = {"timestamp", "lat", "lon", "alt", "vn", "ve", "vd", "roll", "pitch", "yaw"}
    missing = required.difference(df.columns)
    if missing:
        raise RuntimeError(f"truth file missing columns {sorted(missing)}: {path}")
    return df


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_mounting_base_rpy_deg(config: dict[str, Any]) -> np.ndarray:
    fusion_cfg = config.get("fusion") or {}
    constraints_cfg = fusion_cfg.get("constraints") or {}
    init_cfg = fusion_cfg.get("init") or {}
    cfg_mount = np.array(constraints_cfg.get("imu_mounting_angle", [0.0, 0.0, 0.0]), dtype=float)
    if cfg_mount.size != 3:
        cfg_mount = np.zeros(3, dtype=float)
    mounting_base = cfg_mount.copy()
    use_legacy = bool(init_cfg.get("use_legacy_mounting_base_logic", True))
    init_pitch = float(init_cfg.get("mounting_pitch0", 0.0) or 0.0)
    init_yaw = float(init_cfg.get("mounting_yaw0", 0.0) or 0.0)
    eps = 1e-12
    if use_legacy:
        if abs(init_pitch) > eps:
            mounting_base[1] = 0.0
        if abs(init_yaw) > eps:
            mounting_base[2] = 0.0
    return mounting_base


MOUNTING_MEDIAN_DROP_RATIO = 0.1
MOUNTING_MEDIAN_ROLL_DEG = 0.0
MOUNTING_MEDIAN_SOL_PATHS = {
    "data2": "output/review/EXP-20260310-mounting-median-r1/SOL_data2_baseline_true_iekf_full.txt",
    "data4": "output/review/EXP-20260310-gnss-vel-effect-r1/SOL_data4_vel_with.txt",
}

def detect_dataset_tag(config_path: Path, config: dict[str, Any]) -> str:
    fusion_cfg = config.get("fusion") or {}
    candidates = [
        str(config_path),
        str(fusion_cfg.get("imu_path", "")),
        str(fusion_cfg.get("gnss_path", "")),
        str(fusion_cfg.get("pos_path", "")),
        str(fusion_cfg.get("odo_path", "")),
    ]
    joined = " ".join(candidates).lower()
    if "data2" in joined:
        return "data2"
    if "data4" in joined:
        return "data4"
    return "unknown"


def compute_mounting_median_rpy_deg(sol_path: Path, drop_ratio: float, roll_deg: float) -> np.ndarray:
    sol_df = load_solution_frame(sol_path)
    total = len(sol_df)
    if total <= 0:
        raise RuntimeError(f"empty solution for mounting median: {sol_path}")
    start_idx = int(math.floor(total * drop_ratio))
    if start_idx >= total:
        start_idx = 0
    sel = sol_df.iloc[start_idx:]
    pitch = sel["mounting_pitch"].to_numpy(dtype=float)
    yaw = sel["mounting_yaw"].to_numpy(dtype=float)
    if pitch.size == 0:
        raise RuntimeError(f"no mounting samples after drop_ratio={drop_ratio}: {sol_path}")
    return np.array([roll_deg, float(np.median(pitch)), float(np.median(yaw))], dtype=float)


def resolve_mounting_base_rpy_deg(
    config: dict[str, Any],
    config_path: Path,
    cache: dict[str, np.ndarray],
) -> tuple[np.ndarray, str]:
    dataset_tag = detect_dataset_tag(config_path, config)
    sol_rel = MOUNTING_MEDIAN_SOL_PATHS.get(dataset_tag)
    if sol_rel:
        if dataset_tag not in cache:
            sol_path = repo_path(sol_rel)
            if not sol_path.exists():
                raise RuntimeError(f"mounting median sol not found: {sol_path}")
            cache[dataset_tag] = compute_mounting_median_rpy_deg(
                sol_path,
                MOUNTING_MEDIAN_DROP_RATIO,
                MOUNTING_MEDIAN_ROLL_DEG,
            )
        return cache[dataset_tag], f"median:{dataset_tag}"
    return load_mounting_base_rpy_deg(config), "config"


def resolve_input_path(config_path: Path, raw_path: str) -> Path:
    data_path = Path(str(raw_path))
    if data_path.is_absolute():
        return data_path
    candidate = (REPO_ROOT / data_path).resolve()
    if candidate.exists():
        return candidate
    return (config_path.parent / data_path).resolve()


def extract_truth_path(config: dict[str, Any], config_path: Path) -> Path:
    fusion_cfg = config.get("fusion") or {}
    truth_rel = fusion_cfg.get("pos_path")
    if not truth_rel:
        raise RuntimeError(f"fusion.pos_path missing in config: {config_path}")
    return resolve_input_path(config_path, str(truth_rel))


def extract_gnss_path(config: dict[str, Any], config_path: Path) -> Path | None:
    fusion_cfg = config.get("fusion") or {}
    gnss_rel = fusion_cfg.get("gnss_path")
    if not gnss_rel:
        return None
    return resolve_input_path(config_path, str(gnss_rel))


def extract_split_time_s(config: dict[str, Any], truth: TruthBundle) -> float | None:
    fusion_cfg = config.get("fusion") or {}
    schedule_cfg = fusion_cfg.get("gnss_schedule") or {}
    if not schedule_cfg.get("enabled", False):
        return None
    head_ratio = float(schedule_cfg.get("head_ratio", 0.0) or 0.0)
    if head_ratio <= 0.0:
        return None
    return float((truth.time[-1] - truth.time[0]) * head_ratio)


def load_gnss_timestamps(path: Path) -> np.ndarray:
    has_header = detect_has_header(path)
    df = pd.read_csv(path, sep=r"\s+", header=0 if has_header else None, comment="#", usecols=[0])
    timestamps = df.iloc[:, 0].to_numpy(dtype=float)
    timestamps = timestamps[np.isfinite(timestamps)]
    if timestamps.size == 0:
        return timestamps
    return np.unique(timestamps)


def timestamps_to_windows(timestamps: np.ndarray) -> list[tuple[float, float]]:
    if timestamps.size == 0:
        return []
    if timestamps.size == 1:
        t = float(timestamps[0])
        return [(t, t)]

    dt = np.diff(timestamps)
    positive_dt = dt[dt > 0.0]
    nominal_dt = float(np.median(positive_dt)) if positive_dt.size else 0.0
    # Ignore small timestamp jitter and sporadic sample drops; only long gaps mark GNSS-off windows.
    gap_threshold = max(nominal_dt * 10.0, nominal_dt + 10.0) if nominal_dt > 0.0 else 10.0
    gap_indices = np.where(dt > gap_threshold)[0]
    start_indices = np.concatenate(([0], gap_indices + 1))
    end_indices = np.concatenate((gap_indices, [timestamps.size - 1]))
    extension = nominal_dt if nominal_dt > 0.0 else 0.0

    windows: list[tuple[float, float]] = []
    for start_idx, end_idx in zip(start_indices, end_indices):
        start_t = float(timestamps[start_idx])
        end_t = float(timestamps[end_idx] + extension)
        windows.append((start_t, max(start_t, end_t)))
    return windows


def extract_gnss_windows_s(
    config: dict[str, Any],
    config_path: Path,
    truth: TruthBundle,
    cache: dict[Path, list[tuple[float, float]]],
) -> list[tuple[float, float]]:
    split_time_s = extract_split_time_s(config, truth)
    if split_time_s is not None:
        return [(0.0, split_time_s)]

    gnss_path = extract_gnss_path(config, config_path)
    if gnss_path is None or not gnss_path.exists():
        return []
    if gnss_path not in cache:
        cache[gnss_path] = timestamps_to_windows(load_gnss_timestamps(gnss_path))

    truth_start = float(truth.time[0])
    truth_end = float(truth.time[-1])
    windows_s: list[tuple[float, float]] = []
    for abs_start, abs_end in cache[gnss_path]:
        clipped_start = max(abs_start, truth_start)
        clipped_end = min(abs_end, truth_end)
        if clipped_end < clipped_start:
            continue
        windows_s.append((clipped_start - truth_start, clipped_end - truth_start))
    return windows_s


def select_section_gnss_windows(cases: list[CaseResult]) -> list[tuple[float, float]]:
    for case in cases:
        if len(case.gnss_windows_s) > 1:
            return case.gnss_windows_s
    for case in cases:
        if case.gnss_windows_s:
            return case.gnss_windows_s
    return []


def load_truth_bundle(path: Path, target_points: int) -> TruthBundle:
    truth_df = load_truth_frame(path)
    truth_time = truth_df["timestamp"].to_numpy(dtype=float)
    truth_lat = truth_df["lat"].to_numpy(dtype=float)
    truth_lon = truth_df["lon"].to_numpy(dtype=float)
    truth_alt = truth_df["alt"].to_numpy(dtype=float)
    truth_xyz = llh_deg_to_ecef(truth_lat, truth_lon, truth_alt)
    origin_xyz = truth_xyz[0]
    origin_lat = float(truth_lat[0])
    origin_lon = float(truth_lon[0])
    truth_enu = ecef_to_enu(truth_xyz, origin_xyz, origin_lat, origin_lon)
    truth_plot_df = decimate_df(
        pd.DataFrame(
            {
                "time_sec": truth_time - truth_time[0],
                "east_m": truth_enu[:, 0],
                "north_m": truth_enu[:, 1],
                "up_m": truth_enu[:, 2],
                "alt_m": truth_alt,
            }
        ),
        target_points,
    )
    return TruthBundle(
        time=truth_time,
        time_sec=truth_time - truth_time[0],
        lat_deg=truth_lat,
        lon_deg=truth_lon,
        alt_m=truth_alt,
        vn_mps=truth_df["vn"].to_numpy(dtype=float),
        ve_mps=truth_df["ve"].to_numpy(dtype=float),
        vd_mps=truth_df["vd"].to_numpy(dtype=float),
        roll_deg=truth_df["roll"].to_numpy(dtype=float),
        pitch_deg=truth_df["pitch"].to_numpy(dtype=float),
        yaw_deg=truth_df["yaw"].to_numpy(dtype=float),
        ecef_xyz_m=truth_xyz,
        origin_xyz_m=origin_xyz,
        origin_lat_deg=origin_lat,
        origin_lon_deg=origin_lon,
        trajectory_plot_df=truth_plot_df,
    )


def compute_stats(values: np.ndarray) -> dict[str, float]:
    arr = np.asarray(values, dtype=float)
    abs_arr = np.abs(arr)
    return {
        "mean": float(np.mean(arr)),
        "rmse": float(np.sqrt(np.mean(arr * arr))),
        "p95_abs": float(np.percentile(abs_arr, 95)),
        "max_abs": float(np.max(abs_arr)),
        "final": float(arr[-1]),
    }


def build_case_result(
    spec: CaseSpec,
    truth: TruthBundle,
    target_points: int,
    mounting_base_cache: dict[str, np.ndarray],
    gnss_window_cache: dict[Path, list[tuple[float, float]]],
) -> CaseResult:
    sol_path = repo_path(spec.sol_path)
    cfg_path = repo_path(spec.config_path)
    sol_df = load_solution_frame(sol_path)
    config = load_yaml(cfg_path)
    mounting_base_deg, mounting_base_source = resolve_mounting_base_rpy_deg(config, cfg_path, mounting_base_cache)
    split_time_s = extract_split_time_s(config, truth)
    gnss_windows_s = extract_gnss_windows_s(config, cfg_path, truth, gnss_window_cache)

    sol_time = sol_df["timestamp"].to_numpy(dtype=float)
    sol_time_sec = sol_time - truth.time[0]
    sol_xyz = sol_df[["fused_x", "fused_y", "fused_z"]].to_numpy(dtype=float)
    sol_lat, sol_lon, sol_alt = ecef_to_lla(sol_xyz[:, 0], sol_xyz[:, 1], sol_xyz[:, 2])
    sol_vn, sol_ve, sol_vd = ecef_vel_to_ned(
        sol_df["fused_vx"].to_numpy(dtype=float),
        sol_df["fused_vy"].to_numpy(dtype=float),
        sol_df["fused_vz"].to_numpy(dtype=float),
        sol_lat,
        sol_lon,
    )

    truth_xyz_interp = np.column_stack(
        [interp_linear(sol_time, truth.time, truth.ecef_xyz_m[:, i]) for i in range(3)]
    )
    truth_alt_interp = interp_linear(sol_time, truth.time, truth.alt_m)
    truth_vn_interp = interp_linear(sol_time, truth.time, truth.vn_mps)
    truth_ve_interp = interp_linear(sol_time, truth.time, truth.ve_mps)
    truth_vd_interp = interp_linear(sol_time, truth.time, truth.vd_mps)
    truth_roll_interp = interp_angle_deg(sol_time, truth.time, truth.roll_deg)
    truth_pitch_interp = interp_angle_deg(sol_time, truth.time, truth.pitch_deg)
    truth_yaw_interp = interp_angle_deg(sol_time, truth.time, truth.yaw_deg)

    sol_roll = sol_df["fused_roll"].to_numpy(dtype=float)
    sol_pitch = sol_df["fused_pitch"].to_numpy(dtype=float)
    sol_yaw = sol_df["fused_yaw"].to_numpy(dtype=float)

    body_to_nav = euler_to_rotation(
        np.deg2rad(sol_roll),
        np.deg2rad(sol_pitch),
        np.deg2rad(sol_yaw),
    )
    truth_body_to_nav = euler_to_rotation(
        np.deg2rad(truth_roll_interp),
        np.deg2rad(truth_pitch_interp),
        np.deg2rad(truth_yaw_interp),
    )
    body_roll_err_deg, body_pitch_err_deg, _ = relative_euler_error_deg(
        body_to_nav, truth_body_to_nav
    )

    nav_vel = np.column_stack((sol_vn, sol_ve, sol_vd))
    truth_nav_vel = np.column_stack((truth_vn_interp, truth_ve_interp, truth_vd_interp))
    body_vel = np.einsum("...ji,...j->...i", body_to_nav, nav_vel)
    truth_body_vel = np.einsum("...ji,...j->...i", truth_body_to_nav, truth_nav_vel)

    mount_roll_deg = np.full(sol_time.shape, mounting_base_deg[0], dtype=float)
    mount_pitch_deg = mounting_base_deg[1] + sol_df["mounting_pitch"].to_numpy(dtype=float)
    mount_yaw_deg = mounting_base_deg[2] + sol_df["mounting_yaw"].to_numpy(dtype=float)
    body_to_vehicle = euler_to_rotation(
        np.deg2rad(mount_roll_deg),
        np.deg2rad(mount_pitch_deg),
        np.deg2rad(mount_yaw_deg),
    )
    truth_body_to_vehicle = euler_to_rotation(
        np.deg2rad(np.full(sol_time.shape, mounting_base_deg[0], dtype=float)),
        np.deg2rad(np.full(sol_time.shape, mounting_base_deg[1], dtype=float)),
        np.deg2rad(np.full(sol_time.shape, mounting_base_deg[2], dtype=float)),
    )
    vehicle_vel = np.einsum("...ij,...j->...i", body_to_vehicle, body_vel)
    truth_vehicle_vel = np.einsum("...ij,...j->...i", truth_body_to_vehicle, truth_body_vel)

    vehicle_to_nav = np.einsum("...ij,...jk->...ik", body_to_nav, np.swapaxes(body_to_vehicle, -1, -2))
    truth_vehicle_to_nav = np.einsum(
        "...ij,...jk->...ik", truth_body_to_nav, np.swapaxes(truth_body_to_vehicle, -1, -2)
    )
    _, _, vehicle_heading_err_deg = relative_euler_error_deg(vehicle_to_nav, truth_vehicle_to_nav)

    pos_err_xyz = sol_xyz - truth_xyz_interp
    err3d = np.linalg.norm(pos_err_xyz, axis=1)
    start_idx = int(max(0, math.floor(err3d.size * 0.3)))
    tail = err3d[start_idx:]
    sol_enu = ecef_to_enu(sol_xyz, truth.origin_xyz_m, truth.origin_lat_deg, truth.origin_lon_deg)

    plot_df = pd.DataFrame(
        {
            "time_sec": sol_time_sec,
            "east_m": sol_enu[:, 0],
            "north_m": sol_enu[:, 1],
            "up_m": sol_enu[:, 2],
            "alt_err_m": sol_alt - truth_alt_interp,
            "vn_err_mps": sol_vn - truth_vn_interp,
            "ve_err_mps": sol_ve - truth_ve_interp,
            "vd_err_mps": sol_vd - truth_vd_interp,
            "vv_x_err_mps": vehicle_vel[:, 0] - truth_vehicle_vel[:, 0],
            "vv_y_err_mps": vehicle_vel[:, 1] - truth_vehicle_vel[:, 1],
            "vv_z_err_mps": vehicle_vel[:, 2] - truth_vehicle_vel[:, 2],
            "vehicle_heading_err_deg": vehicle_heading_err_deg,
            "pitch_err_deg": body_pitch_err_deg,
            "roll_err_deg": body_roll_err_deg,
        }
    )

    stats_by_category: dict[str, dict[str, dict[str, float]]] = {}
    for category_key, _, signal_defs in CATEGORY_SPECS:
        stats_by_category[category_key] = {}
        for signal_key, _ in signal_defs:
            stats_by_category[category_key][signal_key] = compute_stats(plot_df[signal_key].to_numpy())

    overview = {
        "case": spec.label,
        "samples": int(err3d.size),
        "rmse_3d_m": float(np.sqrt(np.mean(err3d * err3d))),
        "p95_3d_m": float(np.percentile(err3d, 95)),
        "tail70_rmse_3d_m": float(np.sqrt(np.mean(tail * tail))) if tail.size else float("nan"),
        "final_err_3d_m": float(err3d[-1]),
        "gnss_stop_s": split_time_s,
        "sol_path": spec.sol_path,
        "config_path": spec.config_path,
        "mounting_base_source": mounting_base_source,
        "mounting_base_rpy_deg": [float(x) for x in mounting_base_deg],
    }

    return CaseResult(
        spec=spec,
        overview=overview,
        plot_df=decimate_df(plot_df, target_points),
        stats_by_category=stats_by_category,
        gnss_windows_s=gnss_windows_s,
    )


def build_overview_table(cases: list[CaseResult]) -> pd.DataFrame:
    df = pd.DataFrame([case.overview for case in cases])
    return df[
        [
            "case",
            "rmse_3d_m",
            "p95_3d_m",
            "tail70_rmse_3d_m",
            "final_err_3d_m",
            "samples",
            "gnss_stop_s",
        ]
    ]


def build_stats_tables(cases: list[CaseResult]) -> dict[str, pd.DataFrame]:
    out: dict[str, pd.DataFrame] = {}
    for category_key, _, signal_defs in CATEGORY_SPECS:
        rows: list[dict[str, Any]] = []
        for case in cases:
            for signal_key, signal_label in signal_defs:
                stats = case.stats_by_category[category_key][signal_key]
                rows.append(
                    {
                        "case": case.spec.label,
                        "signal": signal_label,
                        "mean": stats["mean"],
                        "rmse": stats["rmse"],
                        "p95_abs": stats["p95_abs"],
                        "max_abs": stats["max_abs"],
                        "final": stats["final"],
                    }
                )
        out[category_key] = pd.DataFrame(rows)
    return out


def apply_report_style(fig: go.Figure, height: int) -> go.Figure:
    fig.update_layout(
        template="plotly_white",
        paper_bgcolor="#FFFFFF",
        plot_bgcolor="#FFFFFF",
        hovermode="x unified",
        dragmode=False,
        height=height,
        margin=dict(l=70, r=70, t=110, b=60),
        font=dict(family=REPORT_FONT_SANS, size=13, color="#000000"),
        title=dict(
            x=0.5,
            y=0.98,
            xanchor="center",
            yanchor="top",
            font=dict(family=REPORT_FONT_SERIF, size=18, color="#000000"),
        ),
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.03,
            xanchor="center",
            x=0.5,
            bgcolor="rgba(255,255,255,0)",
            bordercolor="rgba(0,0,0,0)",
            borderwidth=0,
            font=dict(size=12, color="#000000"),
        ),
        hoverlabel=dict(
            bgcolor="#FFFFFF",
            bordercolor="#D6D6D6",
            font=dict(family=REPORT_FONT_SANS, size=12, color="#000000"),
        ),
    )
    fig.update_annotations(font=dict(family=REPORT_FONT_SANS, size=13, color="#000000"))
    fig.update_xaxes(
        showgrid=False,
        zeroline=False,
        showline=True,
        linecolor="#000000",
        linewidth=1.5,
        mirror=False,
        ticks="outside",
        tickcolor="#000000",
        tickwidth=1.5,
        ticklen=5,
        title_standoff=10,
        fixedrange=True,
    )
    fig.update_yaxes(
        showgrid=False,
        zeroline=False,
        showline=True,
        linecolor="#000000",
        linewidth=1.5,
        mirror=False,
        ticks="outside",
        tickcolor="#000000",
        tickwidth=1.5,
        ticklen=5,
        title_standoff=10,
        fixedrange=True,
    )
    return fig


def add_split_region(fig: go.Figure, split_time_s: float | None, rows: int) -> None:
    if split_time_s is None:
        return
    for row in range(1, rows + 1):
        fig.add_vline(
            x=split_time_s,
            line_width=0.75,
            line_dash="dash",
            line_color="#A8A8A8",
            row=row,
            col=1,
        )


def add_gnss_window_regions(fig: go.Figure, gnss_windows_s: list[tuple[float, float]], rows: int) -> None:
    for start_s, end_s in gnss_windows_s:
        if end_s <= start_s:
            continue
        for row in range(1, rows + 1):
            fig.add_vrect(
                x0=start_s,
                x1=end_s,
                fillcolor="#DCE9D4",
                opacity=0.35,
                layer="below",
                line_width=0,
                row=row,
                col=1,
            )


def make_trajectory_figure(section: SectionSpec, truth_plot_df: pd.DataFrame, cases: list[CaseResult]) -> go.Figure:
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=truth_plot_df["east_m"],
            y=truth_plot_df["north_m"],
            mode="lines",
            name="真值",
            line=dict(color="#6A6A6A", width=1.1, dash="dash"),
            hovertemplate="真值<br>东向=%{x:.2f} m<br>北向=%{y:.2f} m<extra></extra>",
        )
    )
    for case in cases:
        fig.add_trace(
            go.Scatter(
                x=case.plot_df["east_m"],
                y=case.plot_df["north_m"],
                mode="lines",
                name=case.spec.label,
                line=dict(color=case.spec.color, width=1.15),
                hovertemplate=f"{case.spec.label}<br>东向=%{{x:.2f}} m<br>北向=%{{y:.2f}} m<extra></extra>",
            )
        )
    fig.update_layout(
        title=f"{section.title} · 轨迹对比",
        xaxis_title="东向 [m]",
        yaxis_title="北向 [m]",
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    return apply_report_style(fig, height=520)


def make_error_figure(
    section: SectionSpec,
    cases: list[CaseResult],
    category_title: str,
    signal_defs: list[tuple[str, str]],
    split_time_s: float | None,
    gnss_windows_s: list[tuple[float, float]],
) -> go.Figure:
    rows = len(signal_defs)
    fig = make_subplots(
        rows=rows,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.06,
        subplot_titles=[signal_label for _, signal_label in signal_defs],
    )
    for row_idx, (signal_key, signal_label) in enumerate(signal_defs, start=1):
        for case in cases:
            fig.add_trace(
                go.Scatter(
                    x=case.plot_df["time_sec"],
                    y=case.plot_df[signal_key],
                    mode="lines",
                    name=case.spec.label,
                    legendgroup=case.spec.case_id,
                    showlegend=row_idx == 1,
                    line=dict(color=case.spec.color, width=1.05),
                    hovertemplate=(
                        f"{case.spec.label}<br>t=%{{x:.1f}} s<br>{signal_label}=%{{y:.4f}}<extra></extra>"
                    ),
                ),
                row=row_idx,
                col=1,
            )
        fig.add_hline(y=0.0, line_width=0.65, line_color="#AFAFAF", line_dash="dot", row=row_idx, col=1)
    add_gnss_window_regions(fig, gnss_windows_s, rows)
    add_split_region(fig, split_time_s, rows)
    fig.update_layout(title=f"{section.title} · {category_title}")
    fig.update_xaxes(title_text="时间 [s]", row=rows, col=1)
    return apply_report_style(fig, height=230 + 155 * rows)


def build_section(
    spec: SectionSpec,
    truth_cache: dict[Path, TruthBundle],
    mounting_base_cache: dict[str, np.ndarray],
    gnss_window_cache: dict[Path, list[tuple[float, float]]],
    target_points: int,
) -> SectionResult:
    first_config_path = repo_path(spec.cases[0].config_path)
    first_config = load_yaml(first_config_path)
    truth_path = extract_truth_path(first_config, first_config_path)
    truth = truth_cache.setdefault(truth_path, load_truth_bundle(truth_path, target_points))
    cases = [
        build_case_result(case_spec, truth, target_points, mounting_base_cache, gnss_window_cache)
        for case_spec in spec.cases
    ]
    split_candidates = [case.overview["gnss_stop_s"] for case in cases if case.overview["gnss_stop_s"] is not None]
    split_time_s = split_candidates[0] if split_candidates else None
    gnss_windows_s = select_section_gnss_windows(cases) if spec.shade_gnss_windows else []
    stats_tables = build_stats_tables(cases)
    figures = {
        category_key: make_error_figure(spec, cases, category_title, signal_defs, split_time_s, gnss_windows_s)
        for category_key, category_title, signal_defs in CATEGORY_SPECS
    }
    return SectionResult(
        spec=spec,
        truth_plot_df=truth.trajectory_plot_df,
        cases=cases,
        overview_table=build_overview_table(cases),
        stats_tables=stats_tables,
        split_time_s=split_time_s,
        gnss_windows_s=gnss_windows_s,
        trajectory_fig=make_trajectory_figure(spec, truth.trajectory_plot_df, cases),
        figures=figures,
    )


def build_report_from_sections(section_specs: tuple[SectionSpec, ...] | list[SectionSpec], target_points: int = 4000) -> list[SectionResult]:
    truth_cache: dict[Path, TruthBundle] = {}
    mounting_base_cache: dict[str, np.ndarray] = {}
    gnss_window_cache: dict[Path, list[tuple[float, float]]] = {}
    return [build_section(spec, truth_cache, mounting_base_cache, gnss_window_cache, target_points) for spec in section_specs]


def build_representative_report(target_points: int = 4000) -> list[SectionResult]:
    return build_report_from_sections(REPORT_SECTIONS, target_points=target_points)


def render_section(section: SectionResult) -> None:
    from IPython.display import Markdown, display

    display(Markdown(f"## {section.spec.title}"))
    display(Markdown(section.spec.subtitle))
    display(Markdown("### 轨迹图"))
    section.trajectory_fig.show()
    display(Markdown("### 总览指标"))
    display(section.overview_table.round(3))
    if section.gnss_windows_s:
        note = "背景色表示 GNSS 可用时段。"
    else:
        note = "虚线表示 GNSS 关闭分界线。" if section.split_time_s is not None else "该组不包含 GNSS 关闭分界线。"
    display(Markdown(note))
    for category_key, category_title, _ in CATEGORY_SPECS:
        display(Markdown(f"### {category_title}"))
        section.figures[category_key].show()
        display(section.stats_tables[category_key].round(4))


def render_report(sections: list[SectionResult] | None = None) -> list[SectionResult]:
    built_sections = sections or build_representative_report()
    for section in built_sections:
        render_section(section)
    return built_sections


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build the representative navigation report data summary.")
    parser.add_argument("--target-points", type=int, default=4000, help="Max points kept per trace for Plotly figures.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sections = build_representative_report(target_points=args.target_points)
    for section in sections:
        print(f"[{section.spec.title}] {section.spec.subtitle}")
        print(section.overview_table.round(3).to_string(index=False))
        print()


if __name__ == "__main__":
    main()
