from __future__ import annotations

import argparse
from html import escape
from pathlib import Path
import sys

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.interactive_nav_report import CATEGORY_SPECS, build_representative_report


PLOT_CONFIG = {
    "displaylogo": False,
    "displayModeBar": False,
    "responsive": True,
    "scrollZoom": False,
    "doubleClick": False,
}


OVERVIEW_TABLE_RENAME = {
    "case": "实验组",
    "rmse_3d_m": "三维RMSE [m]",
    "p95_3d_m": "三维P95 [m]",
    "tail70_rmse_3d_m": "后70%三维RMSE [m]",
    "final_err_3d_m": "末时刻三维误差 [m]",
    "samples": "样本数",
    "gnss_stop_s": "GNSS 截止时刻 [s]",
}

STATS_TABLE_RENAME = {
    "case": "实验组",
    "signal": "误差通道",
    "mean": "均值",
    "rmse": "RMSE",
    "p95_abs": "|误差| P95",
    "max_abs": "|误差| 最大值",
    "final": "末时刻",
}

ATTR_R2_METRICS = "output/review/EXP-20260312-inekf-mechanism-attribution-r2/metrics.csv"
RATE_R2_METRICS = "output/review/EXP-20260312-inekf-mechanism-rate-vs-weight-r2/metrics.csv"
_CSV_CACHE: dict[str, pd.DataFrame | None] = {}


def _display_df(df, decimals: int = 4, rename_map: dict[str, str] | None = None):
    display_df = df.copy()
    numeric_cols = list(display_df.select_dtypes(include="number").columns)
    if numeric_cols:
        display_df[numeric_cols] = display_df[numeric_cols].round(decimals)
    display_df = display_df.where(display_df.notna(), "—")
    if rename_map:
        display_df = display_df.rename(columns=rename_map)
    return display_df


def dataframe_html(df, decimals: int = 4) -> str:
    return _display_df(df, decimals=decimals).to_html(index=False, border=0, classes="metrics-table", escape=False)


def table_block(df, decimals: int = 4, rename_map: dict[str, str] | None = None) -> str:
    table_df = _display_df(df, decimals=decimals, rename_map=rename_map)
    return f'<div class="table-card">{table_df.to_html(index=False, border=0, classes="metrics-table", escape=False)}</div>'


def format_mounting_source(source: str) -> str:
    if source == "config":
        return "配置文件"
    if source.startswith("median:"):
        dataset = source.split(":", 1)[1]
        return f"{dataset} 中位数基准"
    if source == "unknown":
        return "未标注"
    return source


def mounting_note_block(section) -> str:
    items: list[str] = []
    for case in section.cases:
        overview = case.overview or {}
        rpy = overview.get("mounting_base_rpy_deg")
        source = overview.get("mounting_base_source", "unknown")
        if not rpy or len(rpy) != 3:
            continue
        roll, pitch, yaw = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
        items.append(
            f"<li><strong>{escape(case.spec.label)}</strong>："
            f"来源 {escape(format_mounting_source(str(source)))}，"
            f"RPY [deg] = ({roll:.6f}, {pitch:.6f}, {yaw:.6f})</li>"
        )
    if not items:
        return ""
    return (
        '<div class="note-card">'
        '<div class="note-title">v系真值基准安装角说明</div>'
        '<ul class="note-list">'
        + "".join(items)
        + "</ul></div>"
    )


def _fmt(val: float | None, decimals: int = 3) -> str:
    if val is None:
        return "nan"
    try:
        return f"{float(val):.{decimals}f}"
    except (TypeError, ValueError):
        return "nan"


def _ratio(a: float | None, b: float | None) -> str:
    try:
        if b is None or float(b) == 0.0:
            return "nan"
        return f"{float(a) / float(b):.2f}x"
    except (TypeError, ValueError, ZeroDivisionError):
        return "nan"


def _case_by_id(section, case_id: str):
    for case in section.cases:
        if case.spec.case_id == case_id:
            return case
    return None


def _overview_metric(case, key: str) -> float | None:
    if case is None:
        return None
    return case.overview.get(key)


def _load_metrics_csv(rel_path: str) -> pd.DataFrame | None:
    if rel_path in _CSV_CACHE:
        return _CSV_CACHE[rel_path]
    path = REPO_ROOT / rel_path
    if not path.exists():
        _CSV_CACHE[rel_path] = None
        return None
    _CSV_CACHE[rel_path] = pd.read_csv(path, encoding="utf-8-sig")
    return _CSV_CACHE[rel_path]


def _metrics_row(rel_path: str, case_id: str):
    df = _load_metrics_csv(rel_path)
    if df is None or df.empty or "case_id" not in df.columns:
        return None
    subset = df[df["case_id"] == case_id]
    if subset.empty:
        return None
    return subset.iloc[0]


def _improvement_pct(best: float | None, baseline: float | None) -> str:
    try:
        if baseline is None or float(baseline) == 0.0:
            return "nan"
        return f"{(1.0 - float(best) / float(baseline)) * 100.0:.1f}%"
    except (TypeError, ValueError, ZeroDivisionError):
        return "nan"


def _analysis_card(title: str, items: list[str]) -> str:
    if not items:
        return ""
    return (
        '<div class="analysis-card">'
        f'<div class="analysis-title">{escape(title)}</div>'
        '<ul class="analysis-list">'
        + "".join(f"<li>{item}</li>" for item in items)
        + "</ul></div>"
    )


def analysis_block(section) -> str:
    if section.spec.exp_id == "EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1":
        baseline_case = _case_by_id(section, "data2_baseline_eskf_fresh")
        outage_case = _case_by_id(section, "data2_true_tuned_gnss_outage_cycle")

        baseline_rmse = _overview_metric(baseline_case, "rmse_3d_m")
        outage_rmse = _overview_metric(outage_case, "rmse_3d_m")
        baseline_final = _overview_metric(baseline_case, "final_err_3d_m")
        outage_final = _overview_metric(outage_case, "final_err_3d_m")
        baseline_tail = _overview_metric(baseline_case, "tail70_rmse_3d_m")
        outage_tail = _overview_metric(outage_case, "tail70_rmse_3d_m")

        items = [
            (
                "这一页已经用 corrected 配置替换掉旧的 raw-NHC `inekf_best` 版本。"
                f"fresh 对照组 `data2 全GNSS ESKF` 的三维 RMSE 为 <strong>{_fmt(baseline_rmse)} m</strong>；"
                f"实验组改为当前正式最优的 `data2 true_iekf tuned` 配置"
                f"（`ODO raw + NHC 0.75Hz`），并将 GNSS 改为 `900 s 收敛 + 300/120 s` 周期断续后，"
                f"三维 RMSE 为 <strong>{_fmt(outage_rmse)} m</strong>。"
            ),
            (
                f"末时刻三维误差由 <strong>{_fmt(baseline_final)} m</strong> 降到 "
                f"<strong>{_fmt(outage_final)} m</strong>；后 70% 轨迹的三维 RMSE 也由 "
                f"<strong>{_fmt(baseline_tail)} m</strong> 降到 <strong>{_fmt(outage_tail)} m</strong>。"
            ),
            (
                "也就是说，真正的 tuned true_iekf 最优配置在这份周期断续 GNSS 文件下并没有退化，反而比 full-GNSS ESKF 控制组略优；"
                "此前出现“周期断续 InEKF 明显更差”的结论，根因是第十一组最初误接到了旧 `inekf_best + raw NHC` 口径。"
            ),
            (
                "这一组仍然不是纯粹的单一算法 A/B：控制组是 full-GNSS ESKF，实验组同时改变了滤波结构、NHC 调度和 GNSS 调度。"
                "它回答的是“当前 tuned true_iekf 最优配置换成同一份周期断续 GNSS 文件后，整体表现会怎样”。"
            ),
        ]
        return _analysis_card("结果分析", items)

    if section.spec.exp_id == "EXP-20260310-inekf-mechanism-group7":
        base = _metrics_row(ATTR_R2_METRICS, "data4_true_working")
        proc = _metrics_row(ATTR_R2_METRICS, "data4_true_working_proc_eskf")
        vel = _metrics_row(ATTR_R2_METRICS, "data4_true_working_veljac_eskf")
        reset = _metrics_row(ATTR_R2_METRICS, "data4_true_working_reset_off")
        combo = _metrics_row(ATTR_R2_METRICS, "data4_true_working_proc_eskf_reset_off")
        raw = _metrics_row(RATE_R2_METRICS, "data4_true_working")
        ported = _metrics_row(RATE_R2_METRICS, "data4_true_ported_075hz")
        weight = _metrics_row(RATE_R2_METRICS, "data4_true_raw_weight")
        if base is None or proc is None or vel is None or reset is None:
            return ""
        items = [
            (
                f"补充机制归因实验表明，`data4 GNSS30 true_iekf` 的工作点三维 RMSE 为 "
                f"<strong>{_fmt(base.get('rmse_3d_m'))} m</strong>，`v系航向误差` 线性漂移斜率约为 "
                f"<strong>{_fmt(base.get('heading_slope_deg_per_ks'))} deg/ks</strong>；"
                f"把过程模型强行切回 ESKF 后，RMSE 退化到 <strong>{_fmt(proc.get('rmse_3d_m'))} m</strong>，"
                f"而把 `ODO/NHC` 的 velocity Jacobian 切回 ESKF 几乎不变 "
                f"(<strong>{_fmt(vel.get('rmse_3d_m'))} m</strong>)。"
            ),
            (
                f"单独关闭 reset Gamma 后，RMSE 退化到 <strong>{_fmt(reset.get('rmse_3d_m'))} m</strong>；"
                f"若同时做 `process=eskf + reset=I`，RMSE 为 <strong>{_fmt(combo.get('rmse_3d_m'))} m</strong>。"
                if combo is not None
                else f"单独关闭 reset Gamma 后，RMSE 退化到 <strong>{_fmt(reset.get('rmse_3d_m'))} m</strong>。"
            ),
            (
                "因此，当前 codepath 下真正主导 `v系航向误差` 改善的不是 `ODO/NHC Jacobian` 写法本身，"
                "而是 Lie-core 的时间更新与 reset 协方差重映射一致性；Jacobian 形式更像是在维持坐标一致，而不是单独贡献主要性能提升。"
            ),
        ]
        if raw is not None and ported is not None and weight is not None:
            items.append(
                f"补充的 data4 移植对照里，把 data2 的 `0.75 Hz` 低频策略直接搬过来会得到 "
                f"<strong>{_fmt(ported.get('rmse_3d_m'))} m</strong>，raw + rate-equivalent `scaled R` 得到 "
                f"<strong>{_fmt(weight.get('rmse_3d_m'))} m</strong>，都不优于 raw/raw 的 "
                f"<strong>{_fmt(raw.get('rmse_3d_m'))} m</strong>；这说明“降频≈减权”的机理可以迁移，但最优频率本身仍是 dataset-specific。"
            )
        return _analysis_card("补充机制归因", items)

    if section.spec.exp_id == "EXP-20260310-inekf-mechanism-group8":
        base = _metrics_row(ATTR_R2_METRICS, "data2_true_tuned")
        proc = _metrics_row(ATTR_R2_METRICS, "data2_true_tuned_proc_eskf")
        vel = _metrics_row(ATTR_R2_METRICS, "data2_true_tuned_veljac_eskf")
        reset = _metrics_row(ATTR_R2_METRICS, "data2_true_tuned_reset_off")
        combo = _metrics_row(ATTR_R2_METRICS, "data2_true_tuned_proc_eskf_reset_off")
        if base is None or proc is None or vel is None or reset is None:
            return ""
        items = [
            (
                f"补充机制归因实验中，`data2 GNSS30 true_iekf` 在当前最优 `NHC 0.75 Hz` 下的三维 RMSE 为 "
                f"<strong>{_fmt(base.get('rmse_3d_m'))} m</strong>，`v系航向误差` 漂移斜率约为 "
                f"<strong>{_fmt(base.get('heading_slope_deg_per_ks'))} deg/ks</strong>。"
            ),
            (
                f"把过程模型切回 ESKF 后，RMSE 直接恶化到 <strong>{_fmt(proc.get('rmse_3d_m'))} m</strong>；"
                f"关闭 reset Gamma 后也会恶化到 <strong>{_fmt(reset.get('rmse_3d_m'))} m</strong>；"
                f"但把 `ODO/NHC` Jacobian 切回 ESKF 仅有 <strong>{_fmt(vel.get('rmse_3d_m'))} m</strong>，与基准几乎完全重合。"
            ),
            (
                f"`process=eskf + reset=I` 的联合消融进一步到 <strong>{_fmt(combo.get('rmse_3d_m'))} m</strong>，"
                "说明 process 与 reset 确实共同关键，但主要回归已经由 process mismatch 主导，reset 再叠加一个明显的二级惩罚。"
                if combo is not None
                else "现有证据已经足够把主结论收敛为：process + reset consistency dominate，Jacobian form is near-equivalent on the current codepath。"
            ),
            (
                "因此，这里更合理的原理解释是：InEKF 通过 Lie-core 传播和与其匹配的 reset Gamma，持续抑制了 post-GNSS 段 "
                "`att_z / bg_z / mounting_yaw` 上的错误信息注入，从而显著压低 `v系航向误差`。"
            ),
        ]
        return _analysis_card("补充机制归因", items)

    if section.spec.exp_id == "EXP-20260311-interactive-report-group9":
        raw_case = _case_by_id(section, "data2_gnss30_eskf_nhc_raw")
        nhc_50_case = _case_by_id(section, "data2_gnss30_eskf_nhc_50hz")
        nhc_30_case = _case_by_id(section, "data2_gnss30_eskf_nhc_30hz")
        nhc_1_case = _case_by_id(section, "data2_gnss30_eskf_nhc_1hz")
        rate_weight_case = _metrics_row(RATE_R2_METRICS, "data2_eskf_raw_weight")

        raw_rmse = _overview_metric(raw_case, "rmse_3d_m")
        nhc_50_rmse = _overview_metric(nhc_50_case, "rmse_3d_m")
        nhc_30_rmse = _overview_metric(nhc_30_case, "rmse_3d_m")
        nhc_1_rmse = _overview_metric(nhc_1_case, "rmse_3d_m")

        items = [
            (
                f"以 <strong>raw/raw</strong> 为参考时，`ODO raw + NHC raw` 的三维 RMSE 为 "
                f"<strong>{_fmt(raw_rmse)} m</strong>；仅把 NHC 降到 <strong>50 Hz</strong> 后，RMSE 已下降到 "
                f"<strong>{_fmt(nhc_50_rmse)} m</strong>。"
            ),
            (
                f"细扫结果表明本组最优点落在 <strong>30 Hz</strong>，三维 RMSE 为 "
                f"<strong>{_fmt(nhc_30_rmse)} m</strong>，相对 raw/raw 改善约 "
                f"<strong>{_improvement_pct(nhc_30_rmse, raw_rmse)}</strong>。"
            ),
            (
                f"当 NHC 继续降到 <strong>1 Hz</strong> 时，三维 RMSE 反弹到 "
                f"<strong>{_fmt(nhc_1_rmse)} m</strong>，甚至差于 raw/raw，说明 ESKF 下并不是 "
                f"“NHC 越低频越好”，而是存在一个有限最优频率。"
            ),
            (
                "这一组说明在 `data2 GNSS30 ESKF` 场景下，过高频 NHC 可能约束过强，而过低频又会削弱航向/侧向约束；"
                "当前证据支持把最优区间放在约 <strong>30 Hz</strong> 附近。"
            ),
        ]
        if rate_weight_case is not None:
            items.append(
                f"补充的 `raw + scaled R` 对照进一步把三维 RMSE 压到 <strong>{_fmt(rate_weight_case.get('rmse_3d_m'))} m</strong>，"
                f"`v系航向误差` 斜率降到 <strong>{_fmt(rate_weight_case.get('heading_slope_deg_per_ks'))} deg/ks</strong>；"
                "这说明 data2 ESKF 上“降频有效”的很大一部分，本质上是在降低单位时间 heading 信息注入强度，"
                "而不一定非要通过硬性的时间降采样来实现。"
            )
        return _analysis_card("结果分析", items)

    if section.spec.exp_id == "EXP-20260311-interactive-report-group10":
        raw_case = _case_by_id(section, "data2_gnss30_true_iekf_nhc_raw")
        nhc_10_case = _case_by_id(section, "data2_gnss30_true_iekf_nhc_10hz")
        nhc_1_case = _case_by_id(section, "data2_gnss30_true_iekf_nhc_1hz")
        nhc_075_case = _case_by_id(section, "data2_gnss30_true_iekf_nhc_075hz")
        rate_weight_case = _metrics_row(RATE_R2_METRICS, "data2_true_raw_weight")
        data4_raw = _metrics_row(RATE_R2_METRICS, "data4_true_working")
        data4_ported = _metrics_row(RATE_R2_METRICS, "data4_true_ported_075hz")
        data4_weight = _metrics_row(RATE_R2_METRICS, "data4_true_raw_weight")

        raw_rmse = _overview_metric(raw_case, "rmse_3d_m")
        nhc_10_rmse = _overview_metric(nhc_10_case, "rmse_3d_m")
        nhc_1_rmse = _overview_metric(nhc_1_case, "rmse_3d_m")
        nhc_075_rmse = _overview_metric(nhc_075_case, "rmse_3d_m")

        items = [
            (
                f"本组 raw/raw 的三维 RMSE 为 <strong>{_fmt(raw_rmse)} m</strong>；将 NHC 先降到 "
                f"<strong>10 Hz</strong> 后，RMSE 已降到 <strong>{_fmt(nhc_10_rmse)} m</strong>，再降到 "
                f"<strong>1 Hz</strong> 时进一步降到 <strong>{_fmt(nhc_1_rmse)} m</strong>。"
            ),
            (
                f"细扫到 1 Hz 以下后，当前最优点出现在 <strong>0.75 Hz</strong>，三维 RMSE 为 "
                f"<strong>{_fmt(nhc_075_rmse)} m</strong>，相对 raw/raw 改善约 "
                f"<strong>{_improvement_pct(nhc_075_rmse, raw_rmse)}</strong>。"
            ),
            (
                f"与实验九中 ESKF 的最优 <strong>30 Hz</strong> 相比，本组 true InEKF 的最优频率下探到 "
                f"<strong>0.75 Hz</strong>，相差约 <strong>{_ratio(30.0, 0.75)}</strong>；"
                "说明 NHC 调度最优点对滤波结构高度敏感。"
            ),
            (
                "这一组的趋势不是在中高频附近见顶，而是随着 NHC 持续降频仍显著改善，直到亚赫兹区间才出现最优；"
                "当前证据支持把 `data2 GNSS30 true_iekf` 的 NHC 最优区间放在约 <strong>0.75 Hz</strong>。"
            ),
        ]
        if rate_weight_case is not None:
            items.append(
                f"`raw + scaled R` 的补充对照得到 <strong>{_fmt(rate_weight_case.get('rmse_3d_m'))} m</strong>，"
                "已经非常接近 `0.75 Hz` 的最优点；这说明 true InEKF 上“超低频 NHC 更优”的主要机制，"
                "同样可以解释为单位时间 heading 信息注入被显著压低。"
            )
        if data4_raw is not None and data4_ported is not None and data4_weight is not None:
            items.append(
                f"但把这一组 data2 最优策略直接移植到 data4 时，raw/raw / `0.75 Hz` / `raw + scaled R` 分别为 "
                f"<strong>{_fmt(data4_raw.get('rmse_3d_m'))}</strong> / <strong>{_fmt(data4_ported.get('rmse_3d_m'))}</strong> / "
                f"<strong>{_fmt(data4_weight.get('rmse_3d_m'))} m</strong>；"
                "因此“降频≈减权”的解释可以跨数据集保留，但最优频率本身并不能直接从 data2 迁移到 data4。"
            )
        return _analysis_card("结果分析", items)

    return ""


def render_html(target_points: int, out_path: Path) -> Path:
    sections = build_representative_report(target_points=target_points)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    page_ids = [f"page-{idx}" for idx in range(1, len(sections) + 1)]

    css = """
:root {
  --sidebar-w: 240px;
  --content-max: 1020px;
  --table-max: 960px;
  --bg: #fafafa;
  --surface: #ffffff;
  --border: #d0d0d0;
  --text: #222222;
  --muted: #555555;
  --muted-soft: #888888;
  --sidebar-bg: #ececec;
  --nav-hover: #dfdfdf;
  --nav-active-bg: #d0d0d0;
  --nav-active-text: #111111;
  --font-body: "Times New Roman", "Songti SC", "SimSun", "STSong", serif;
}
*, *::before, *::after { box-sizing: border-box; }
html, body { margin: 0; height: 100%; }
body {
  display: flex;
  background: var(--bg);
  color: var(--text);
  font-family: var(--font-body);
  text-rendering: optimizeLegibility;
}
/* ── Sidebar ── */
.sidebar {
  position: fixed;
  top: 0; left: 0;
  width: var(--sidebar-w);
  height: 100vh;
  overflow-y: auto;
  background: var(--sidebar-bg);
  border-right: 1px solid var(--border);
  display: flex;
  flex-direction: column;
  z-index: 100;
}
.sidebar-header {
  padding: 24px 16px 16px;
  flex-shrink: 0;
  text-align: center;
}
.sidebar-title {
  font-size: 16px;
  font-weight: bold;
  color: var(--text);
  line-height: 1.4;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  padding-bottom: 12px;
  border-bottom: 1px solid var(--border);
}
.sidebar-nav {
  padding: 16px 0;
  flex: 1;
}
.nav-group-label {
  padding: 12px 16px 4px;
  font-size: 11px;
  letter-spacing: 0.15em;
  color: var(--muted-soft);
  text-transform: uppercase;
  user-select: none;
  font-weight: bold;
}
.nav-item {
  display: block;
  padding: 8px 16px;
  font-size: 14px;
  color: var(--text);
  cursor: pointer;
  border: none;
  background: none;
  width: 100%;
  text-align: left;
  line-height: 1.5;
  transition: background 0.15s;
}
.nav-item:hover { background: var(--nav-hover); }
.nav-item.active { background: var(--nav-active-bg); color: var(--nav-active-text); font-weight: bold; }
/* ── Main content ── */
.main-content {
  margin-left: var(--sidebar-w);
  flex: 1;
  min-height: 100vh;
  overflow-y: auto;
  background: var(--bg);
  display: flex;
  justify-content: center;
  align-items: flex-start;
}
.page-container {
  width: 100%;
  max-width: calc(var(--content-max) + 120px);
  background: var(--surface);
  min-height: 100vh;
  padding: 40px 60px 80px;
  box-shadow: 0 0 20px rgba(0, 0, 0, 0.03);
}
.page-view { display: none; }
.page-view.active { display: block; }

/* ── Section elements ── */
.section-header { 
  margin-bottom: 30px; 
  padding-bottom: 20px; 
  border-bottom: 2px solid var(--text); 
  text-align: center;
}
.section-tag { font-size: 12px; letter-spacing: 0.15em; color: var(--muted); text-transform: uppercase; margin-bottom: 8px;}
.section-header h2 { margin: 10px 0; font-size: 28px; font-weight: normal; line-height: 1.3; }
.subtitle { margin: 0; color: var(--muted); font-size: 16px; font-style: italic; }
.meta-row { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 20px; justify-content: center; }
.pill { display: inline-flex; align-items: center; padding: 4px 12px; background: transparent; font-size: 13px; color: var(--muted); font-style: italic; border: none; }

.block-title { margin: 32px 0 16px; font-size: 18px; font-weight: bold; color: var(--text); text-align: center; text-transform: uppercase; letter-spacing: 0.05em; }
.plot-card { margin: 0 auto 16px; width: 100%; max-width: var(--content-max); display: flex; justify-content: center; }
.plot-card .js-plotly-plot, .plot-card .plotly-graph-div { width: 100% !important; margin: 0 auto; }
.plotly .modebar, .modebar { display: none !important; }

.note { text-align: center; margin: 12px auto 24px; color: var(--muted); font-size: 13px; font-style: italic; }
.note-card { border: 1px solid var(--border); background: #fffef9; padding: 12px 14px; border-radius: 6px; margin: 12px auto 16px; max-width: var(--content-max); }
.note-title { font-size: 13px; font-weight: bold; letter-spacing: 0.03em; margin-bottom: 6px; text-align: left; }
.note-list { margin: 0; padding-left: 18px; font-size: 12.5px; color: var(--muted); line-height: 1.5; text-align: left; }

.analysis-card { border: 1px solid var(--border); background: #ffffff; padding: 16px 18px; border-radius: 6px; margin: 24px auto 16px; max-width: var(--content-max); }
.analysis-title { font-size: 14px; font-weight: bold; letter-spacing: 0.04em; margin-bottom: 8px; text-align: left; }
.analysis-list { margin: 0; padding-left: 18px; font-size: 13.5px; color: var(--text); line-height: 1.6; text-align: left; }
.analysis-list li { margin: 6px 0; }

.table-card { margin: 16px auto 32px; width: 100%; max-width: var(--table-max); overflow-x: auto; display: flex; justify-content: center; }
.metrics-table { width: 100%; border-collapse: collapse; text-align: center; }
.metrics-table th, .metrics-table td { padding: 10px 12px; font-size: 14px; line-height: 1.5; color: var(--text); border: none; }
.metrics-table thead tr:first-child th { border-top: 1.5px solid var(--text); border-bottom: 1px solid var(--text); }
.metrics-table tbody tr:last-child td { border-bottom: 1.5px solid var(--text); }
.metrics-table th { font-weight: bold; }
/* First column styling */
.metrics-table th:first-child, .metrics-table td:first-child { text-align: left; }

.module-divider { border: 0; border-top: 1px dashed var(--border); margin: 60px auto; width: 85%; max-width: var(--content-max); }

@media (max-width: 900px) {
  .sidebar { width: 200px; }
  :root { --sidebar-w: 200px; }
  .page-container { padding: 30px 20px 60px; }
}
"""

    js = """
function showPage(pageId) {
  document.querySelectorAll('.page-view').forEach(function(el) {
    el.classList.remove('active');
  });
  document.querySelectorAll('.nav-item').forEach(function(el) {
    el.classList.remove('active');
  });
  var page = document.getElementById(pageId);
  if (page) page.classList.add('active');
  var btn = document.querySelector('.nav-item[data-page="' + pageId + '"]');
  if (btn) btn.classList.add('active');
  if (page) page.scrollIntoView ? page.parentElement.scrollTop = 0 : null;
  var main = document.querySelector('.main-content');
  if (main) main.scrollTop = 0;
}
document.addEventListener('DOMContentLoaded', function() {
  showPage('""" + page_ids[0] + """');
});
"""

    # Build group labels for sidebar
    GROUP_LABELS = [
        (1, 3, "基准对比实验"),
        (4, 8, "问题定向实验"),
        (9, 10, "NHC频率实验"),
        (11, 11, "GNSS断续扩展"),
    ]

    def group_label_for(idx: int) -> str | None:
        for start, end, label in GROUP_LABELS:
            if idx == start:
                return label
        return None

    parts: list[str] = [
        '<!DOCTYPE html>',
        '<html lang="zh-CN">',
        '<head>',
        '<meta charset="utf-8" />',
        '<meta name="viewport" content="width=device-width, initial-scale=1" />',
        '<title>组合导航代表性实验交互报告</title>',
        f'<style>{css}</style>',
        '</head>',
        '<body>',
        '<!-- Sidebar -->',
        '<aside class="sidebar">',
        '<div class="sidebar-header">',
        '<div class="sidebar-eyebrow">交互式报告</div>',
        '<div class="sidebar-title">组合导航<br>代表性实验报告</div>',
        '</div>',
        '<nav class="sidebar-nav">',
    ]

    for idx, (section, pid) in enumerate(zip(sections, page_ids), start=1):
        gl = group_label_for(idx)
        if gl:
            parts.append(f'<div class="nav-group-label">{escape(gl)}</div>')
        short_title = escape(section.spec.title)
        parts.append(
            f'<button class="nav-item" data-page="{pid}" onclick="showPage(\'{pid}\')">'
            f'{idx}. {short_title}'
            f'</button>'
        )

    parts.extend([
        '</nav>',
        '</aside>',
        '<!-- Main content -->',
        '<div class="main-content">',
        '<div class="page-container">',
    ])

    include_plotly = True
    for idx, (section, pid) in enumerate(zip(sections, page_ids), start=1):
        parts.extend([
            f'<div class="page-view" id="{pid}">',
            '<div class="section-header">',
            f'<div class="section-tag">实验 {idx:02d}</div>',
            f'<h2>{escape(section.spec.title)}</h2>',
            f'<p class="subtitle">{escape(section.spec.subtitle)}</p>',
            '<div class="meta-row">',
        ])
        if section.split_time_s is not None:
            parts.append(f'<span class="pill">GNSS 截止时刻: {section.split_time_s:.3f} s</span>')
        parts.append(f'<span class="pill">实验组数量: {len(section.cases)}</span>')
        parts.extend([
            '</div>',
            '</div>',
            '<h3 class="block-title">轨迹图</h3>',
            '<div class="plot-card">',
        ])
        parts.append(
            section.trajectory_fig.to_html(
                full_html=False,
                include_plotlyjs='inline' if include_plotly else False,
                config=PLOT_CONFIG,
            )
        )
        parts.append('</div>')
        include_plotly = False
        parts.append('<h3 class="block-title">总览指标</h3>')
        parts.append(table_block(section.overview_table, decimals=3, rename_map=OVERVIEW_TABLE_RENAME))
        note_block = mounting_note_block(section)
        if note_block:
            parts.append(note_block)
        parts.append(
            '<p class="note">虚线表示 GNSS 关闭分界线。</p>'
            if section.split_time_s is not None
            else '<p class="note">该组不包含 GNSS 关闭分界线。</p>'
        )
        parts.append('<hr class="module-divider" />')

        for i, (category_key, category_title, _) in enumerate(CATEGORY_SPECS):
            parts.append(f'<h3 class="block-title">{escape(category_title)}</h3>')
            parts.append('<div class="plot-card">')
            parts.append(
                section.figures[category_key].to_html(
                    full_html=False,
                    include_plotlyjs=False,
                    config=PLOT_CONFIG,
                )
            )
            parts.append('</div>')
            parts.append(table_block(section.stats_tables[category_key], rename_map=STATS_TABLE_RENAME))
            
            if i < len(CATEGORY_SPECS) - 1:
                parts.append('<hr class="module-divider" />')
        analysis = analysis_block(section)
        if analysis:
            parts.append(analysis)
        parts.extend([
            '</div>',  # .page-view
        ])

    parts.extend([
        '</div>',  # .page-container
        '</div>',  # .main-content
        f'<script>{js}</script>',
        '</body>',
        '</html>',
    ])
    out_path.write_text('\n'.join(parts), encoding='utf-8')
    return out_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Export a self-contained HTML report for the representative navigation experiments.')
    parser.add_argument('--target-points', type=int, default=2500, help='Maximum points per trace after decimation.')
    parser.add_argument('--out', type=Path, default=Path('output/html/representative_navigation_interactive_report.html'), help='Output HTML file path.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_path = render_html(target_points=args.target_points, out_path=args.out)
    print(out_path.resolve())


if __name__ == '__main__':
    main()
