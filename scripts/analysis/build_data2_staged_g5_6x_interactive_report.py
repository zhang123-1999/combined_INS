from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.io as pio
from plotly.offline.offline import get_plotlyjs
from plotly.subplots import make_subplots

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.interactive_nav_report import apply_report_style, decimate_df, euler_to_rotation, relative_euler_error_deg  # noqa: E402
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import json_safe  # noqa: E402
from scripts.analysis.run_nhc_state_convergence_research import (  # noqa: E402
    build_motion_frame,
    build_truth_interp,
    load_imu_dataframe,
    load_pos_dataframe,
    merge_case_outputs,
    wrap_deg,
)


EXP_ID_DEFAULT = "EXP-20260326-data2-staged-g5-6x-interactive-report-r1"
INPUT_DIR_DEFAULT = Path("output/data2_staged_g5_no_imu_scale_r2_20260325")
OUTPUT_HTML_DEFAULT = INPUT_DIR_DEFAULT / "interactive" / "staged_g5_6x_report.html"
OUTPUT_MANIFEST_DEFAULT = INPUT_DIR_DEFAULT / "interactive" / "staged_g5_6x_report_manifest.json"
MAIN_CASE_ID_DEFAULT = "staged_g5_odo_nhc_noise_6x"
CONTROL_CASE_ID_DEFAULT = "staged_g5_odo_nhc_noise_6x_phase3_ins_only"
TARGET_POINTS_DEFAULT = 4000
GRID_COLUMNS = 3
AXIS_TICKFORMAT = ".6g"
HOVER_NUMBER_FORMAT = ".6g"
WORKING_TOTAL_MOUNTING_DEG = {"roll": 0.0, "pitch": 0.36, "yaw": 0.84}
CASE_COLORS = {
    MAIN_CASE_ID_DEFAULT: "#0B5FA5",
    CONTROL_CASE_ID_DEFAULT: "#8F2D2D",
}
PLOTLY_EMBED_CONFIG = {
    "responsive": True,
    "displaylogo": False,
    "modeBarButtonsToRemove": [
        "lasso2d",
        "select2d",
        "autoScale2d",
        "toggleSpikelines",
    ],
}


@dataclass(frozen=True)
class SignalSpec:
    key: str
    label: str
    unit: str
    truth_column: str | None
    plot_mode: str = "value"
    show_truth: bool = True


@dataclass(frozen=True)
class SignalGroup:
    group_id: str
    title: str
    states: tuple[SignalSpec, ...]


@dataclass(frozen=True)
class CaseArtifacts:
    case_id: str
    label: str
    color: str
    config_path: Path
    sol_path: Path
    state_series_path: Path
    all_states_path: Path
    all_states_mtime: str
    sol_mtime: str
    state_series_mtime: str


@dataclass(frozen=True)
class ExperimentContext:
    input_dir: Path
    manifest_path: Path
    manifest: dict[str, Any]
    base_config_path: Path
    base_config: dict[str, Any]
    truth_reference_path: Path
    truth_reference: dict[str, Any]
    case_metrics_df: pd.DataFrame
    phase_metrics_df: pd.DataFrame
    phase1_start_time: float
    phase1_end_time: float
    phase2_end_time: float
    phase3_start_time: float
    final_time: float
    gnss_off_windows: list[tuple[float, float]]
    main_case: CaseArtifacts
    control_case: CaseArtifacts


NON_PVA_STATE_SPECS: tuple[SignalSpec, ...] = (
    SignalSpec("ba_x_mgal", "ba_x", "mGal", "truth_ba_x_mgal", show_truth=False),
    SignalSpec("ba_y_mgal", "ba_y", "mGal", "truth_ba_y_mgal", show_truth=False),
    SignalSpec("ba_z_mgal", "ba_z", "mGal", "truth_ba_z_mgal", show_truth=False),
    SignalSpec("bg_x_degh", "bg_x", "deg/h", "truth_bg_x_degh", show_truth=False),
    SignalSpec("bg_y_degh", "bg_y", "deg/h", "truth_bg_y_degh", show_truth=False),
    SignalSpec("bg_z_degh", "bg_z", "deg/h", "truth_bg_z_degh", show_truth=False),
    SignalSpec("odo_scale_state", "odo_scale", "1", "truth_odo_scale_state", plot_mode="delta", show_truth=False),
    SignalSpec("mounting_roll_deg", "mount_total_roll", "deg", "truth_mounting_roll_deg", plot_mode="delta", show_truth=False),
    SignalSpec("mounting_pitch_deg", "mount_total_pitch", "deg", "truth_mounting_pitch_deg", plot_mode="delta", show_truth=False),
    SignalSpec("mounting_yaw_deg", "mount_total_yaw", "deg", "truth_mounting_yaw_deg", plot_mode="delta", show_truth=False),
    SignalSpec("odo_lever_x_m", "odo_lever_x", "m", "truth_odo_lever_x_m", plot_mode="delta", show_truth=False),
    SignalSpec("odo_lever_y_m", "odo_lever_y", "m", "truth_odo_lever_y_m", plot_mode="delta", show_truth=False),
    SignalSpec("odo_lever_z_m", "odo_lever_z", "m", "truth_odo_lever_z_m", plot_mode="delta", show_truth=False),
    SignalSpec("gnss_lever_x_m", "gnss_lever_x", "m", "truth_gnss_lever_x_m", plot_mode="delta", show_truth=False),
    SignalSpec("gnss_lever_y_m", "gnss_lever_y", "m", "truth_gnss_lever_y_m", plot_mode="delta", show_truth=False),
    SignalSpec("gnss_lever_z_m", "gnss_lever_z", "m", "truth_gnss_lever_z_m", plot_mode="delta", show_truth=False),
)

PVA_ERROR_GROUP_SPECS: tuple[SignalGroup, ...] = (
    SignalGroup(
        "position",
        "P 误差",
        (
            SignalSpec("p_n_m", "p_n", "m", "truth_p_n_m"),
            SignalSpec("p_e_m", "p_e", "m", "truth_p_e_m"),
            SignalSpec("p_u_m", "p_u", "m", "truth_p_u_m"),
        ),
    ),
    SignalGroup(
        "velocity",
        "V 误差",
        (
            SignalSpec("v_n_mps", "v_n", "m/s", "truth_v_n_mps"),
            SignalSpec("v_e_mps", "v_e", "m/s", "truth_v_e_mps"),
            SignalSpec("v_u_mps", "v_u", "m/s", "truth_v_u_mps"),
        ),
    ),
    SignalGroup(
        "attitude",
        "A 误差",
        (
            SignalSpec("roll_deg", "roll", "deg", "truth_roll_deg"),
            SignalSpec("pitch_deg", "pitch", "deg", "truth_pitch_deg"),
            SignalSpec("yaw_deg", "yaw", "deg", "truth_yaw_deg"),
        ),
    ),
)
CONTROL_PVA_GROUP_SPECS = PVA_ERROR_GROUP_SPECS

VEHICLE_VELOCITY_ERROR_SPECS: tuple[SignalSpec, ...] = (
    SignalSpec("vv_x_err_mps", "v_v.x err", "m/s", None),
    SignalSpec("vv_y_err_mps", "v_v.y err", "m/s", None),
    SignalSpec("vv_z_err_mps", "v_v.z err", "m/s", None),
)
VEHICLE_HEADING_ERROR_SPEC = SignalSpec("vehicle_heading_err_deg", "vehicle heading err", "deg", None)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build the staged G5 6x interactive Plotly report.")
    parser.add_argument("--input-dir", type=Path, default=INPUT_DIR_DEFAULT)
    parser.add_argument("--main-case-id", default=MAIN_CASE_ID_DEFAULT)
    parser.add_argument("--control-case-id", default=CONTROL_CASE_ID_DEFAULT)
    parser.add_argument("--output-html", type=Path, default=OUTPUT_HTML_DEFAULT)
    parser.add_argument("--output-manifest", type=Path, default=OUTPUT_MANIFEST_DEFAULT)
    parser.add_argument("--target-points", type=int, default=TARGET_POINTS_DEFAULT)
    parser.add_argument("--phase3-only-start", type=float, default=None)
    args = parser.parse_args()
    args.input_dir = (REPO_ROOT / args.input_dir).resolve()
    args.output_html = (REPO_ROOT / args.output_html).resolve()
    args.output_manifest = (REPO_ROOT / args.output_manifest).resolve()
    return args


def format_metric(value: Any, digits: int = 6) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (float, int, np.floating)):
        value = float(value)
        if not math.isfinite(value):
            return "NA"
        return f"{value:.{digits}f}"
    return str(value)


def rel_seconds(timestamps: pd.Series | np.ndarray, start_time: float) -> np.ndarray:
    return np.asarray(timestamps, dtype=float) - float(start_time)


def build_vehicle_truth_reference(truth_reference: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(truth_reference)
    out.setdefault("sources", {}).setdefault("mounting_total_truth", {})["value_deg"] = dict(WORKING_TOTAL_MOUNTING_DEG)
    return out


def build_case_artifacts(row: pd.Series, case_id: str) -> CaseArtifacts:
    color = CASE_COLORS.get(case_id, "#4C78A8")
    return CaseArtifacts(
        case_id=case_id,
        label=str(row["case_label"]),
        color=color,
        config_path=(REPO_ROOT / str(row["config_path"])).resolve(),
        sol_path=(REPO_ROOT / str(row["sol_path"])).resolve(),
        state_series_path=(REPO_ROOT / str(row["state_series_path"])).resolve(),
        all_states_path=(REPO_ROOT / str(row["all_states_path"])).resolve(),
        all_states_mtime=str(row.get("all_states_mtime", "")),
        sol_mtime=str(row.get("sol_mtime", "")),
        state_series_mtime=str(row.get("state_series_mtime", "")),
    )


def load_experiment_context(input_dir: Path, main_case_id: str, control_case_id: str) -> ExperimentContext:
    manifest_path = (input_dir / "manifest.json").resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    base_config_path = (REPO_ROOT / manifest["base_config"]).resolve()
    truth_reference_path = (REPO_ROOT / manifest["truth_reference_path"]).resolve()
    base_config = load_yaml(base_config_path)
    truth_reference = json.loads(truth_reference_path.read_text(encoding="utf-8"))
    case_metrics_df = pd.read_csv((input_dir / "case_metrics.csv").resolve())
    phase_metrics_df = pd.read_csv((input_dir / "phase_metrics.csv").resolve())

    main_row = case_metrics_df.loc[case_metrics_df["case_id"] == main_case_id].iloc[0]
    control_row = case_metrics_df.loc[case_metrics_df["case_id"] == control_case_id].iloc[0]
    case_meta = manifest["case_metadata"][main_case_id]
    phase1_end_time = float(case_meta["phase1_end_time"])
    phase2_end_time = float(case_meta["phase2_end_time"])
    phase3_start_time = phase2_end_time
    final_time = float(manifest["phase_windows"]["phase3"][1])
    return ExperimentContext(
        input_dir=input_dir,
        manifest_path=manifest_path,
        manifest=manifest,
        base_config_path=base_config_path,
        base_config=base_config,
        truth_reference_path=truth_reference_path,
        truth_reference=truth_reference,
        case_metrics_df=case_metrics_df,
        phase_metrics_df=phase_metrics_df,
        phase1_start_time=float(manifest["phase_windows"]["phase1"][0]),
        phase1_end_time=phase1_end_time,
        phase2_end_time=phase2_end_time,
        phase3_start_time=phase3_start_time,
        final_time=final_time,
        gnss_off_windows=[(float(start), float(end)) for start, end in case_meta["gnss_off_windows"]],
        main_case=build_case_artifacts(main_row, main_case_id),
        control_case=build_case_artifacts(control_row, control_case_id),
    )


def load_all_states(case: CaseArtifacts, start_time: float) -> pd.DataFrame:
    df = pd.read_csv(case.all_states_path)
    df["time_rel_s"] = rel_seconds(df["timestamp"], start_time)
    return df


def build_vehicle_error_frame(context: ExperimentContext, case: CaseArtifacts, target_points: int) -> pd.DataFrame:
    merged_df = merge_case_outputs(case.sol_path, case.state_series_path)
    truth_df = load_pos_dataframe((REPO_ROOT / context.base_config["fusion"]["pos_path"]).resolve())
    truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
    imu_df = load_imu_dataframe((REPO_ROOT / context.base_config["fusion"]["imu_path"]).resolve())
    vehicle_truth = build_vehicle_truth_reference(context.truth_reference)
    motion_df = build_motion_frame(merged_df, truth_interp_df, imu_df, vehicle_truth)

    fused_roll = np.deg2rad(merged_df["fused_roll"].to_numpy(dtype=float))
    fused_pitch = np.deg2rad(merged_df["fused_pitch"].to_numpy(dtype=float))
    fused_yaw = np.deg2rad(merged_df["fused_yaw"].to_numpy(dtype=float))
    truth_roll = np.deg2rad(truth_interp_df["roll"].to_numpy(dtype=float))
    truth_pitch = np.deg2rad(truth_interp_df["pitch"].to_numpy(dtype=float))
    truth_yaw = np.deg2rad(truth_interp_df["yaw"].to_numpy(dtype=float))

    body_to_nav = euler_to_rotation(fused_roll, fused_pitch, fused_yaw)
    truth_body_to_nav = euler_to_rotation(truth_roll, truth_pitch, truth_yaw)
    body_to_vehicle = euler_to_rotation(
        np.deg2rad(merged_df["total_mounting_roll_deg"].to_numpy(dtype=float)),
        np.deg2rad(merged_df["total_mounting_pitch_deg"].to_numpy(dtype=float)),
        np.deg2rad(merged_df["total_mounting_yaw_deg"].to_numpy(dtype=float)),
    )
    truth_body_to_vehicle = euler_to_rotation(
        np.full(len(merged_df), math.radians(WORKING_TOTAL_MOUNTING_DEG["roll"]), dtype=float),
        np.full(len(merged_df), math.radians(WORKING_TOTAL_MOUNTING_DEG["pitch"]), dtype=float),
        np.full(len(merged_df), math.radians(WORKING_TOTAL_MOUNTING_DEG["yaw"]), dtype=float),
    )
    vehicle_to_nav = np.einsum("...ij,...jk->...ik", body_to_nav, np.swapaxes(body_to_vehicle, -1, -2))
    truth_vehicle_to_nav = np.einsum(
        "...ij,...jk->...ik",
        truth_body_to_nav,
        np.swapaxes(truth_body_to_vehicle, -1, -2),
    )
    _, _, vehicle_heading_err_deg = relative_euler_error_deg(vehicle_to_nav, truth_vehicle_to_nav)

    all_states = load_all_states(case, context.phase1_start_time)
    plot_df = pd.DataFrame(
        {
            "timestamp": merged_df["timestamp"].to_numpy(dtype=float),
            "time_rel_s": rel_seconds(merged_df["timestamp"], context.phase1_start_time),
            "vv_x_err_mps": motion_df["v_v_x_mps"].to_numpy(dtype=float)
            - motion_df["truth_v_v_x_mps"].to_numpy(dtype=float),
            "vv_y_err_mps": motion_df["v_v_y_mps"].to_numpy(dtype=float)
            - motion_df["truth_v_v_y_mps"].to_numpy(dtype=float),
            "vv_z_err_mps": motion_df["v_v_z_mps"].to_numpy(dtype=float)
            - motion_df["truth_v_v_z_mps"].to_numpy(dtype=float),
            "vehicle_heading_err_deg": vehicle_heading_err_deg,
            "p_e_m": all_states["p_e_m"].to_numpy(dtype=float),
            "p_n_m": all_states["p_n_m"].to_numpy(dtype=float),
            "truth_p_e_m": all_states["truth_p_e_m"].to_numpy(dtype=float),
            "truth_p_n_m": all_states["truth_p_n_m"].to_numpy(dtype=float),
        }
    )
    return decimate_df(plot_df, target_points)


def select_window(df: pd.DataFrame, start_time_abs: float | None = None) -> pd.DataFrame:
    if start_time_abs is None:
        return df
    return df.loc[df["timestamp"] >= float(start_time_abs)].reset_index(drop=True)


def signal_error(df: pd.DataFrame, spec: SignalSpec) -> np.ndarray:
    if spec.truth_column is None:
        return df[spec.key].to_numpy(dtype=float)
    values = df[spec.key].to_numpy(dtype=float)
    truth = df[spec.truth_column].to_numpy(dtype=float)
    if spec.unit == "deg":
        return wrap_deg(values - truth)
    return values - truth


def signal_delta(df: pd.DataFrame, spec: SignalSpec) -> np.ndarray:
    values = df[spec.key].to_numpy(dtype=float)
    baseline = float(values[0])
    if spec.unit == "deg":
        return wrap_deg(values - baseline)
    return values - baseline


def state_plot_values(df: pd.DataFrame, spec: SignalSpec) -> np.ndarray:
    if spec.plot_mode == "error":
        return signal_error(df, spec)
    if spec.plot_mode == "delta":
        return signal_delta(df, spec)
    return df[spec.key].to_numpy(dtype=float)


def state_plot_title(spec: SignalSpec) -> str:
    if spec.plot_mode == "error":
        return f"{spec.label} err [{spec.unit}]"
    if spec.plot_mode == "delta":
        return f"Δ{spec.label} [{spec.unit}]"
    return f"{spec.label} [{spec.unit}]"


def apply_ieee_figure_style(fig: go.Figure, height: int) -> go.Figure:
    fig = apply_report_style(fig, height=height)
    fig.update_layout(
        height=height,
        paper_bgcolor="#FFFFFF",
        plot_bgcolor="#FFFFFF",
        margin=dict(l=52, r=18, t=56, b=40),
        hovermode="x unified",
        font=dict(family='"Times New Roman", "Songti SC", "SimSun", "STSong", serif', size=12, color="#111111"),
        title=dict(
            x=0.5,
            y=0.975,
            xanchor="center",
            yanchor="top",
            font=dict(
                family='"Times New Roman", "Songti SC", "SimSun", "STSong", serif',
                size=15,
                color="#111111",
            ),
        ),
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.01,
            xanchor="right",
            x=1.0,
            bgcolor="rgba(255,255,255,0)",
            borderwidth=0,
            font=dict(
                family='"Times New Roman", "Songti SC", "SimSun", "STSong", serif',
                size=10,
                color="#111111",
            ),
        ),
        hoverlabel=dict(
            bgcolor="#FFFFFF",
            bordercolor="#C8C8C8",
            font=dict(
                family='"Times New Roman", "Songti SC", "SimSun", "STSong", serif',
                size=11,
                color="#111111",
            ),
        ),
    )
    fig.update_annotations(
        font=dict(family='"Times New Roman", "Songti SC", "SimSun", "STSong", serif', size=11, color="#111111")
    )
    fig.update_xaxes(
        showgrid=False,
        linecolor="#111111",
        linewidth=1.0,
        tickcolor="#111111",
        tickwidth=1.0,
        ticklen=4,
        ticks="outside",
        title_standoff=8,
        tickfont=dict(size=10),
        title_font=dict(size=11),
    )
    fig.update_yaxes(
        showgrid=True,
        gridcolor="#E1E1E1",
        gridwidth=0.55,
        linecolor="#111111",
        linewidth=1.0,
        tickcolor="#111111",
        tickwidth=1.0,
        ticklen=4,
        ticks="outside",
        title_standoff=8,
        tickfont=dict(size=10),
        title_font=dict(size=11),
        tickformat=AXIS_TICKFORMAT,
        exponentformat="e",
        showexponent="all",
    )
    return fig


def add_time_regions(fig: go.Figure, context: ExperimentContext, rows: int, show_phase_boundaries: bool = True) -> None:
    off_windows_rel = [
        (start - context.phase1_start_time, end - context.phase1_start_time)
        for start, end in context.gnss_off_windows
    ]
    for row in range(1, rows + 1):
        for start_time, end_time in off_windows_rel:
            fig.add_vrect(
                x0=start_time,
                x1=end_time,
                fillcolor="#ECECEC",
                opacity=0.5,
                layer="below",
                line_width=0,
                row=row,
                col=1,
            )
        if show_phase_boundaries:
            fig.add_vline(
                x=context.phase1_end_time - context.phase1_start_time,
                line_width=0.8,
                line_dash="dash",
                line_color="#7A7A7A",
                row=row,
                col=1,
            )
            fig.add_vline(
                x=context.phase2_end_time - context.phase1_start_time,
                line_width=0.8,
                line_dash="dash",
                line_color="#7A7A7A",
                row=row,
                col=1,
            )


def build_state_signal_figure(
    df: pd.DataFrame,
    spec: SignalSpec,
    context: ExperimentContext,
    color: str,
    target_points: int,
) -> go.Figure:
    columns = ["time_rel_s", spec.key]
    if spec.truth_column:
        columns.append(spec.truth_column)
    data = decimate_df(df[columns], target_points)
    series = state_plot_values(data, spec)
    if spec.plot_mode == "error":
        trace_name = "error"
        hover_value_label = "err"
    elif spec.plot_mode == "delta":
        trace_name = "delta"
        hover_value_label = f"Δ{spec.label}"
    else:
        trace_name = "estimate"
        hover_value_label = spec.label
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=data["time_rel_s"],
            y=series,
            mode="lines",
            name=trace_name,
            line=dict(color=color, width=1.25),
            hovertemplate=(
                f"t=%{{x:.1f}} s<br>{hover_value_label}=%{{y:{HOVER_NUMBER_FORMAT}}} {spec.unit}<extra></extra>"
            ),
        )
    )
    if spec.truth_column and spec.show_truth and spec.plot_mode == "value":
        fig.add_trace(
            go.Scatter(
                x=data["time_rel_s"],
                y=data[spec.truth_column],
                mode="lines",
                name="truth",
                line=dict(color="#111111", width=0.95, dash="dash"),
                hovertemplate=f"t=%{{x:.1f}} s<br>truth=%{{y:{HOVER_NUMBER_FORMAT}}} {spec.unit}<extra></extra>",
            )
        )
    add_time_regions(fig, context, rows=1, show_phase_boundaries=True)
    if spec.plot_mode != "value":
        fig.add_hline(y=0.0, line_width=0.65, line_dash="dot", line_color="#9C9C9C")
    fig.update_layout(title=state_plot_title(spec))
    fig.update_xaxes(title_text="时间 [s]")
    fig.update_yaxes(title_text=spec.unit)
    return apply_ieee_figure_style(fig, height=264)


def build_error_group_figure(
    df: pd.DataFrame,
    group: SignalGroup,
    context: ExperimentContext,
    color: str,
    show_phase_boundaries: bool,
    target_points: int,
) -> go.Figure:
    keep_cols = ["timestamp", "time_rel_s"] + [item.key for item in group.states] + [
        item.truth_column for item in group.states if item.truth_column
    ]
    data = decimate_df(df[keep_cols], target_points)
    fig = make_subplots(
        rows=len(group.states),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.06,
        subplot_titles=[f"{state.label} [{state.unit}]" for state in group.states],
    )
    for idx, state in enumerate(group.states, start=1):
        fig.add_trace(
            go.Scatter(
                x=data["time_rel_s"],
                y=signal_error(data, state),
                mode="lines",
                name=state.label,
                showlegend=False,
                line=dict(color=color, width=1.15),
                hovertemplate=f"{state.label}<br>t=%{{x:.1f}} s<br>err=%{{y:{HOVER_NUMBER_FORMAT}}} {state.unit}<extra></extra>",
            ),
            row=idx,
            col=1,
        )
        fig.add_hline(y=0.0, line_width=0.65, line_dash="dot", line_color="#9C9C9C", row=idx, col=1)
    add_time_regions(fig, context, rows=len(group.states), show_phase_boundaries=show_phase_boundaries)
    fig.update_layout(title=group.title)
    fig.update_xaxes(title_text="时间 [s]", row=len(group.states), col=1)
    return apply_ieee_figure_style(fig, height=220 + 120 * len(group.states))


def build_vehicle_velocity_figure(df: pd.DataFrame, context: ExperimentContext, color: str) -> go.Figure:
    fig = make_subplots(
        rows=len(VEHICLE_VELOCITY_ERROR_SPECS),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.06,
        subplot_titles=[f"{spec.label} [{spec.unit}]" for spec in VEHICLE_VELOCITY_ERROR_SPECS],
    )
    for idx, spec in enumerate(VEHICLE_VELOCITY_ERROR_SPECS, start=1):
        fig.add_trace(
            go.Scatter(
                x=df["time_rel_s"],
                y=df[spec.key],
                mode="lines",
                name=spec.label,
                showlegend=False,
                line=dict(color=color, width=1.15),
                hovertemplate=f"{spec.label}<br>t=%{{x:.1f}} s<br>err=%{{y:{HOVER_NUMBER_FORMAT}}} {spec.unit}<extra></extra>",
            ),
            row=idx,
            col=1,
        )
        fig.add_hline(y=0.0, line_width=0.65, line_dash="dot", line_color="#9C9C9C", row=idx, col=1)
    add_time_regions(fig, context, rows=len(VEHICLE_VELOCITY_ERROR_SPECS), show_phase_boundaries=True)
    fig.update_layout(title="v 系速度误差")
    fig.update_xaxes(title_text="时间 [s]", row=len(VEHICLE_VELOCITY_ERROR_SPECS), col=1)
    return apply_ieee_figure_style(fig, height=220 + 120 * len(VEHICLE_VELOCITY_ERROR_SPECS))


def build_vehicle_heading_figure(df: pd.DataFrame, context: ExperimentContext, color: str) -> go.Figure:
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=df["time_rel_s"],
            y=df[VEHICLE_HEADING_ERROR_SPEC.key],
            mode="lines",
            name=VEHICLE_HEADING_ERROR_SPEC.label,
            line=dict(color=color, width=1.2),
            hovertemplate=f"t=%{{x:.1f}} s<br>heading err=%{{y:{HOVER_NUMBER_FORMAT}}} deg<extra></extra>",
        )
    )
    add_time_regions(fig, context, rows=1, show_phase_boundaries=True)
    fig.add_hline(y=0.0, line_width=0.65, line_dash="dot", line_color="#9C9C9C")
    fig.update_layout(
        title=f"v 系航向误差 [deg] (truth yaw={WORKING_TOTAL_MOUNTING_DEG['yaw']:.2f} deg)"
    )
    fig.update_xaxes(title_text="时间 [s]")
    fig.update_yaxes(title_text="deg")
    return apply_ieee_figure_style(fig, height=320)


def build_trajectory_figure(df: pd.DataFrame, color: str) -> go.Figure:
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=df["truth_p_e_m"],
            y=df["truth_p_n_m"],
            mode="lines",
            name="truth",
            line=dict(color="#111111", width=0.95, dash="dash"),
            hovertemplate="truth<br>east=%{x:.3f} m<br>north=%{y:.3f} m<extra></extra>",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=df["p_e_m"],
            y=df["p_n_m"],
            mode="lines",
            name="6x",
            line=dict(color=color, width=1.2),
            hovertemplate="6x<br>east=%{x:.3f} m<br>north=%{y:.3f} m<extra></extra>",
        )
    )
    fig.update_layout(title="轨迹图", xaxis_title="East [m]", yaxis_title="North [m]")
    fig.update_yaxes(scaleanchor="x", scaleratio=1.0)
    return apply_ieee_figure_style(fig, height=320)


def summary_rows(context: ExperimentContext) -> list[dict[str, str]]:
    case_metrics = context.case_metrics_df.set_index("case_id")
    phase_metrics = context.phase_metrics_df.set_index(["case_id", "window_name"])
    main = case_metrics.loc[context.main_case.case_id]
    control = case_metrics.loc[context.control_case.case_id]

    def phase_row(case_id: str, window_name: str) -> pd.Series:
        return phase_metrics.loc[(case_id, window_name)]

    main_phase2 = phase_row(context.main_case.case_id, "phase2_ins_gnss_odo_nhc")
    main_phase3 = phase_row(context.main_case.case_id, "phase3_periodic_gnss_outage")
    main_out01 = phase_row(context.main_case.case_id, "gnss_off_01")
    control_phase3 = phase_row(context.control_case.case_id, "phase3_periodic_gnss_outage")
    control_out01 = phase_row(context.control_case.case_id, "gnss_off_01")
    return [
        {
            "case": context.main_case.label,
            "overall_rmse_3d_m": format_metric(main["overall_rmse_3d_m_aux"]),
            "phase2_rmse_3d_m": format_metric(main_phase2["rmse_3d_m"]),
            "phase3_rmse_3d_m": format_metric(main_phase3["rmse_3d_m"]),
            "gnss_off_01_rmse_3d_m": format_metric(main_out01["rmse_3d_m"]),
            "gnss_off_01_final_err_3d_m": format_metric(main_out01["final_err_3d_m"]),
        },
        {
            "case": context.control_case.label,
            "overall_rmse_3d_m": format_metric(control["overall_rmse_3d_m_aux"]),
            "phase2_rmse_3d_m": format_metric(control["phase2_rmse_3d_m"]),
            "phase3_rmse_3d_m": format_metric(control_phase3["rmse_3d_m"]),
            "gnss_off_01_rmse_3d_m": format_metric(control_out01["rmse_3d_m"]),
            "gnss_off_01_final_err_3d_m": format_metric(control_out01["final_err_3d_m"]),
        },
    ]


def render_summary_table(context: ExperimentContext) -> str:
    rows = summary_rows(context)
    header = """
<table class="summary-table">
  <thead>
    <tr>
      <th>case</th>
      <th>overall RMSE3D [m]</th>
      <th>phase2 RMSE3D [m]</th>
      <th>phase3 RMSE3D [m]</th>
      <th>gnss_off_01 RMSE3D [m]</th>
      <th>gnss_off_01 final err 3D [m]</th>
    </tr>
  </thead>
  <tbody>
"""
    body = []
    for row in rows:
        body.append(
            "<tr>"
            f"<td>{row['case']}</td>"
            f"<td>{row['overall_rmse_3d_m']}</td>"
            f"<td>{row['phase2_rmse_3d_m']}</td>"
            f"<td>{row['phase3_rmse_3d_m']}</td>"
            f"<td>{row['gnss_off_01_rmse_3d_m']}</td>"
            f"<td>{row['gnss_off_01_final_err_3d_m']}</td>"
            "</tr>"
        )
    return header + "\n".join(body) + "\n  </tbody>\n</table>"


def render_freshness_list(context: ExperimentContext) -> str:
    items = [
        ("exp_id", context.manifest["exp_id"]),
        ("report_exp_id", EXP_ID_DEFAULT),
        ("base_config", rel_from_root(context.base_config_path, REPO_ROOT)),
        ("input_manifest", rel_from_root(context.manifest_path, REPO_ROOT)),
        ("truth_reference", rel_from_root(context.truth_reference_path, REPO_ROOT)),
        ("main_all_states", rel_from_root(context.main_case.all_states_path, REPO_ROOT)),
        ("control_all_states", rel_from_root(context.control_case.all_states_path, REPO_ROOT)),
        ("main_all_states_mtime", context.main_case.all_states_mtime),
        ("control_all_states_mtime", context.control_case.all_states_mtime),
        (
            "vehicle_truth_mounting_deg",
            f"roll={WORKING_TOTAL_MOUNTING_DEG['roll']:.2f}, "
            f"pitch={WORKING_TOTAL_MOUNTING_DEG['pitch']:.2f}, "
            f"yaw={WORKING_TOTAL_MOUNTING_DEG['yaw']:.2f}",
        ),
    ]
    return '<ul class="freshness-list">' + "".join(
        f"<li><b>{key}</b>: <code>{value}</code></li>" for key, value in items
    ) + "</ul>"


def figure_block_html(fig: go.Figure, section_id: str, index: int) -> str:
    div_id = f"{section_id}-fig-{index}"
    fig_html = pio.to_html(
        fig,
        include_plotlyjs=False,
        full_html=False,
        div_id=div_id,
        config=PLOTLY_EMBED_CONFIG,
    )
    return f'<div class="plot-card">{fig_html}</div>'


def render_html_page(
    context: ExperimentContext,
    main_states: pd.DataFrame,
    main_vehicle: pd.DataFrame,
    control_states_phase3: pd.DataFrame,
    target_points: int,
) -> str:
    state_cards = [
        figure_block_html(
            build_state_signal_figure(main_states, spec, context, context.main_case.color, target_points),
            "state",
            idx,
        )
        for idx, spec in enumerate(NON_PVA_STATE_SPECS, start=1)
    ]
    pva_cards = [
        figure_block_html(
            build_error_group_figure(
                main_states,
                group,
                context,
                context.main_case.color,
                show_phase_boundaries=True,
                target_points=target_points,
            ),
            "main-pva",
            idx,
        )
        for idx, group in enumerate(PVA_ERROR_GROUP_SPECS, start=1)
    ]
    main_vehicle_cards = [
        figure_block_html(build_vehicle_velocity_figure(main_vehicle, context, context.main_case.color), "vehicle", 1),
        figure_block_html(build_vehicle_heading_figure(main_vehicle, context, context.main_case.color), "vehicle", 2),
        figure_block_html(build_trajectory_figure(main_vehicle, context.main_case.color), "vehicle", 3),
    ]
    control_cards = [
        figure_block_html(
            build_error_group_figure(
                control_states_phase3,
                group,
                context,
                context.control_case.color,
                show_phase_boundaries=False,
                target_points=target_points,
            ),
            "control-pva",
            idx,
        )
        for idx, group in enumerate(CONTROL_PVA_GROUP_SPECS, start=1)
    ]

    styles = """
:root {
  --paper: #FFFFFF;
  --ink: #111111;
  --muted: #4F4F4F;
  --rule: #B9B9B9;
  --ieee-blue: #0B5FA5;
  --ieee-red: #8F2D2D;
  --ieee-gold: #9B6A00;
  --shade: #F4F4F4;
}
html, body { margin: 0; padding: 0; background: var(--paper); color: var(--ink); font-family: "Times New Roman", "Songti SC", "SimSun", "STSong", serif; }
body { line-height: 1.35; }
.page { max-width: 1840px; margin: 0 auto; padding: 12px 14px 22px 14px; }
.hero { padding: 0 0 8px 0; border-bottom: 1.6px solid var(--ink); }
.hero h1 { margin: 0; font-size: 28px; font-weight: 600; letter-spacing: 0.01em; }
.hero p { margin: 4px 0 0 0; font-size: 14px; color: var(--muted); }
.toc { margin-top: 6px; font-size: 13px; }
.toc a { color: var(--ieee-blue); text-decoration: none; margin-right: 14px; }
.section { margin-top: 10px; padding-top: 8px; border-top: 1px solid var(--rule); }
.section h2 { margin: 0 0 4px 0; font-size: 20px; font-weight: 600; }
.section p { margin: 0 0 6px 0; font-size: 13px; color: var(--muted); }
.plot-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 6px; align-items: start; }
.plot-card { min-width: 0; background: transparent; border: 0; border-radius: 0; padding: 0; box-shadow: none; }
.plot-card .js-plotly-plot, .plot-card .plotly-graph-div { width: 100%; }
.summary-table { width: 100%; border-collapse: collapse; margin: 4px 0 8px 0; font-size: 13px; }
.summary-table th, .summary-table td { border: 1px solid var(--rule); padding: 4px 6px; text-align: center; }
.summary-table th { background: #F8F8F8; font-weight: 600; }
.freshness-list { list-style: none; columns: 2; column-gap: 20px; padding: 0; margin: 6px 0 0 0; font-size: 13px; }
.freshness-list li { margin: 0 0 4px 0; break-inside: avoid; }
code { background: var(--shade); padding: 0 4px; }
@media (max-width: 1380px) { .plot-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); } .freshness-list { columns: 1; } }
@media (max-width: 860px) { .page { padding: 10px 10px 18px 10px; } .plot-grid { grid-template-columns: 1fr; gap: 8px; } }
"""

    plotly_js = get_plotlyjs()
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Staged G5 6x Interactive Report</title>
  <style>{styles}</style>
  <script>{plotly_js}</script>
</head>
<body>
  <div class="page">
    <section class="hero">
      <h1>Staged G5 6x 交互图册</h1>
      <p>主结果使用 <code>{context.main_case.case_id}</code>；第三阶段无 ODO/NHC 对照使用 <code>{context.control_case.case_id}</code>。</p>
      <p>版式按单页论文图册整理：紧凑排版、白底黑字、IEEE 配色，便于横向对照。</p>
      <div class="toc">
        <a href="#summary">摘要</a>
        <a href="#state25">非PVA状态</a>
        <a href="#pva6x">6x PVA误差</a>
        <a href="#vehicle">v系与轨迹</a>
        <a href="#control">phase3 无ODO/NHC</a>
      </div>
    </section>

    <section id="summary" class="section">
      <h2>摘要与 Freshness</h2>
      <p>阶段边界沿用原实验：phase1 结束 <code>{context.phase1_end_time:.1f}</code>，phase2/phase3 分界 <code>{context.phase2_end_time:.1f}</code>。浅灰底纹表示 phase3 GNSS outage 窗口。</p>
      {render_summary_table(context)}
      {render_freshness_list(context)}
    </section>

    <section id="state25" class="section">
      <h2>6x 非 PVA 状态变化（16 维）</h2>
      <p>零偏仍画估计值本身；ODO/NHC 标定状态统一按相对初值变化绘制，即首个样本归零后再显示后续变化，原始首值仍保留 case config / solver 初值而不是被改写成 0；sg/sa 已移除，因为当前 `G5` 下它们全程固定为零；中间安装角状态项也不再单独展示。</p>
      <div class="plot-grid">
        {''.join(state_cards)}
      </div>
    </section>

    <section id="pva6x" class="section">
      <h2>6x PVA 误差</h2>
      <p>位置、速度、姿态各自成组展示，便于直接对照 phase2 和 phase3 窗口表现。</p>
      <div class="plot-grid">
        {''.join(pva_cards)}
      </div>
    </section>

    <section id="vehicle" class="section">
      <h2>6x v 系速度 / v 系航向 / 轨迹</h2>
      <p>v 系相关 truth 口径固定采用工作真值 <code>pitch=0.36 deg</code>, <code>yaw=0.84 deg</code>。</p>
      <div class="plot-grid">
        {''.join(main_vehicle_cards)}
      </div>
    </section>

    <section id="control" class="section">
      <h2>第三阶段不加 ODO/NHC 的 PVA 误差图</h2>
      <p>该区只显示 <code>{context.control_case.case_id}</code> 的 phase3 窗口，phase1/phase2 已裁掉。</p>
      <div class="plot-grid">
        {''.join(control_cards)}
      </div>
    </section>
  </div>
</body>
</html>
"""


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    context = load_experiment_context(args.input_dir, args.main_case_id, args.control_case_id)
    phase3_only_start = float(args.phase3_only_start) if args.phase3_only_start is not None else context.phase3_start_time

    main_states = decimate_df(load_all_states(context.main_case, context.phase1_start_time), args.target_points)
    main_vehicle = build_vehicle_error_frame(context, context.main_case, args.target_points)
    control_states = load_all_states(context.control_case, context.phase1_start_time)
    control_states_phase3 = decimate_df(select_window(control_states, phase3_only_start), args.target_points)

    ensure_dir(args.output_html.parent)
    html = render_html_page(context, main_states, main_vehicle, control_states_phase3, args.target_points)
    args.output_html.write_text(html, encoding="utf-8")

    output_manifest = {
        "exp_id": EXP_ID_DEFAULT,
        "source_exp_id": context.manifest["exp_id"],
        "input_dir": rel_from_root(context.input_dir, REPO_ROOT),
        "input_manifest": rel_from_root(context.manifest_path, REPO_ROOT),
        "output_html": rel_from_root(args.output_html, REPO_ROOT),
        "phase3_only_start": phase3_only_start,
        "target_points": int(args.target_points),
        "main_case_id": context.main_case.case_id,
        "control_case_id": context.control_case.case_id,
        "vehicle_truth_mounting_deg": dict(WORKING_TOTAL_MOUNTING_DEG),
        "artifacts": {
            "main_all_states": rel_from_root(context.main_case.all_states_path, REPO_ROOT),
            "control_all_states": rel_from_root(context.control_case.all_states_path, REPO_ROOT),
            "main_sol": rel_from_root(context.main_case.sol_path, REPO_ROOT),
            "main_state_series": rel_from_root(context.main_case.state_series_path, REPO_ROOT),
            "control_sol": rel_from_root(context.control_case.sol_path, REPO_ROOT),
            "control_state_series": rel_from_root(context.control_case.state_series_path, REPO_ROOT),
        },
        "freshness": {
            "main_all_states_mtime": context.main_case.all_states_mtime,
            "control_all_states_mtime": context.control_case.all_states_mtime,
            "base_config_mtime": context.manifest["freshness"]["base_config_mtime"],
            "source_case_metrics_mtime": context.manifest["freshness"]["case_metrics_mtime"],
            "source_phase_metrics_mtime": context.manifest["freshness"]["phase_metrics_mtime"],
        },
    }
    ensure_dir(args.output_manifest.parent)
    args.output_manifest.write_text(json.dumps(json_safe(output_manifest), ensure_ascii=False, indent=2), encoding="utf-8")
    return output_manifest


def main() -> None:
    args = parse_args()
    manifest = build_report(args)
    print(manifest["output_html"])


if __name__ == "__main__":
    main()
