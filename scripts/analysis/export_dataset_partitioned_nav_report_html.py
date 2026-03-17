from __future__ import annotations

import argparse
import base64
import mimetypes
import os
import shutil
from html import escape
from pathlib import Path
import sys

import pandas as pd
from plotly.offline import get_plotlyjs

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.dataset_partitioned_nav_report import (
    STATE_PLOT_LABELS,
    STATE_PLOT_ORDER,
    DatasetPartitionedReport,
    DatasetReportPage,
    StateGallerySpec,
    build_dataset_partitioned_report,
)
from scripts.analysis.interactive_nav_report import CATEGORY_SPECS


PLOT_CONFIG = {
    "displaylogo": False,
    "displayModeBar": False,
    "responsive": True,
    "scrollZoom": False,
    "doubleClick": False,
}
PLOTLY_ASSET_NAME = "plotly-6.5.2.min.js"

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


def _section_timing_note(page: DatasetReportPage) -> str:
    section = page.section_result
    if section is None:
        return ""
    if section.gnss_windows_s:
        return '<p class="note">背景色表示 GNSS 可用时段。</p>'
    if section.split_time_s is not None:
        return '<p class="note">虚线表示 GNSS 关闭分界线。</p>'
    return '<p class="note">该组不包含 GNSS 关闭分界线。</p>'


def _display_df(df: pd.DataFrame, decimals: int = 4, rename_map: dict[str, str] | None = None) -> pd.DataFrame:
    display_df = df.copy()
    numeric_cols = list(display_df.select_dtypes(include="number").columns)
    if numeric_cols:
        display_df[numeric_cols] = display_df[numeric_cols].round(decimals)
    display_df = display_df.where(display_df.notna(), "—")
    if rename_map:
        display_df = display_df.rename(columns=rename_map)
    return display_df


def table_block(
    df: pd.DataFrame,
    decimals: int = 4,
    rename_map: dict[str, str] | None = None,
    *,
    card_class: str = "table-card",
    table_class: str = "metrics-table",
) -> str:
    table_df = _display_df(df, decimals=decimals, rename_map=rename_map)
    return f'<div class="{escape(card_class)}">{table_df.to_html(index=False, border=0, classes=table_class, escape=False)}</div>'


def _render_intro_sections(page: DatasetReportPage) -> str:
    if not page.intro_sections:
        return ""
    parts: list[str] = []
    for section in page.intro_sections:
        parts.append('<section class="reader-section">')
        parts.append(f'<h3 class="reader-section-title">{escape(section.title)}</h3>')
        if section.body_html:
            parts.append(section.body_html)
        for table in section.tables:
            parts.append(f'<div class="reader-table-title">{escape(table.title)}</div>')
            card_class = f"table-card intro-table-card intro-table-card--{escape(table.table_kind)}"
            table_class = f"metrics-table intro-table intro-table--{escape(table.table_kind)}"
            parts.append(
                table_block(
                    table.dataframe,
                    decimals=4,
                    card_class=card_class,
                    table_class=table_class,
                )
            )
            if table.note_html:
                parts.append(table.note_html)
        parts.append("</section>")
    return "".join(parts)


def _rel_href(target: Path, out_path: Path) -> str:
    return Path(os.path.relpath(target, out_path.parent)).as_posix()


def _file_data_uri(path: Path) -> str:
    mime_type, _ = mimetypes.guess_type(path.name)
    payload = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type or 'application/octet-stream'};base64,{payload}"


def _prepare_gallery_asset_dir(gallery: StateGallerySpec, out_path: Path, asset_dirs: dict[str, Path]) -> Path:
    source_dir = (REPO_ROOT / gallery.image_dir).resolve()
    cache_key = str(source_dir)
    if cache_key in asset_dirs:
        return asset_dirs[cache_key]
    safe_dir = out_path.parent / "_report_assets" / f"gallery_{len(asset_dirs) + 1}"
    safe_dir.mkdir(parents=True, exist_ok=True)
    for filename in STATE_PLOT_ORDER:
        source_path = source_dir / filename
        if source_path.exists():
            shutil.copy2(source_path, safe_dir / filename)
    asset_dirs[cache_key] = safe_dir
    return safe_dir


def _gallery_block(
    gallery: StateGallerySpec,
    out_path: Path,
    anchor_id: str,
    asset_dirs: dict[str, Path],
    *,
    inline_assets: bool,
) -> str:
    source_dir = (REPO_ROOT / gallery.image_dir).resolve()
    image_dir = None if inline_assets else _prepare_gallery_asset_dir(gallery, out_path, asset_dirs)
    figures: list[str] = []
    for filename in STATE_PLOT_ORDER:
        image_path = (source_dir / filename) if inline_assets else (image_dir / filename)
        if not image_path.exists():
            continue
        href = _file_data_uri(image_path) if inline_assets else _rel_href(image_path, out_path)
        caption = STATE_PLOT_LABELS.get(filename, filename)
        figures.append(
            '<figure class="gallery-item">'
            f'<img src="{escape(href)}" alt="{escape(gallery.label)} - {escape(caption)}" loading="lazy" />'
            f'<figcaption>{escape(caption)}</figcaption>'
            "</figure>"
        )
    return (
        f'<div class="gallery-card" id="{escape(anchor_id)}">'
        f'<div class="gallery-title">{escape(gallery.label)}</div>'
        f'<div class="gallery-grid">{"".join(figures)}</div>'
        "</div>"
    )


def _render_state_section(
    page: DatasetReportPage,
    out_path: Path,
    seen: dict[tuple[str, str], str],
    asset_dirs: dict[str, Path],
    *,
    inline_assets: bool,
) -> str:
    if not page.state_galleries:
        return ""
    parts: list[str] = ['<h3 class="block-title">状态量变化图</h3>']
    for gallery in page.state_galleries:
        dedupe_key = (page.group_id, gallery.dedupe_key)
        if dedupe_key in seen:
            parts.append(
                '<div class="gallery-ref">'
                f'{escape(gallery.label)} 的状态图已在本数据集前文展示，'
                f'<a href="#{escape(seen[dedupe_key])}">点击跳转到首次展示位置</a>。'
                "</div>"
            )
            continue
        anchor_id = f"gallery-{page.group_id}-{len(seen) + 1}"
        seen[dedupe_key] = anchor_id
        parts.append(_gallery_block(gallery, out_path, anchor_id, asset_dirs, inline_assets=inline_assets))
    return "".join(parts)


def _ensure_plotly_asset(out_path: Path) -> Path:
    asset_dir = out_path.parent / "_report_assets"
    asset_dir.mkdir(parents=True, exist_ok=True)
    plotly_asset = asset_dir / PLOTLY_ASSET_NAME
    if not plotly_asset.exists():
        plotly_asset.write_text(get_plotlyjs(), encoding="utf-8")
    return plotly_asset


def _plotly_tag(out_path: Path, *, inline_assets: bool) -> str:
    if inline_assets:
        plotly_js = get_plotlyjs().replace("</script>", "<\\/script>")
        return f"<script>{plotly_js}</script>"
    plotly_asset = _ensure_plotly_asset(out_path)
    return f'<script src="{escape(_rel_href(plotly_asset, out_path))}"></script>'


def render_html(target_points: int, out_path: Path, *, inline_assets: bool = True) -> Path:
    report: DatasetPartitionedReport = build_dataset_partitioned_report(target_points=target_points)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    css = """
:root {
  --sidebar-w: 15.625rem;
  --content-max: 65rem;
  --table-max: 78rem;
  --bg: #f7f5f0;
  --surface: #ffffff;
  --border: #d5d0c5;
  --text: #222222;
  --muted: #5b5b5b;
  --muted-soft: #8a857b;
  --sidebar-bg: #ece6da;
  --nav-hover: #e2dbce;
  --nav-active-bg: #d3cab9;
  --nav-active-text: #111111;
  --font-body: "Times New Roman", "Songti SC", "SimSun", "STSong", serif;
}
*, *::before, *::after { box-sizing: border-box; }
html, body { margin: 0; min-height: 100%; }
body {
  display: flex;
  flex-direction: column;
  background: var(--bg);
  color: var(--text);
  font-family: var(--font-body);
  text-rendering: optimizeLegibility;
}
.sidebar {
  position: sticky;
  top: 0;
  width: 100%;
  max-height: 45vh;
  overflow-y: auto;
  background: var(--sidebar-bg);
  border-bottom: 1px solid var(--border);
  display: flex;
  flex-direction: column;
  z-index: 100;
}
.sidebar-header {
  padding: 1rem 1rem 0.75rem;
  flex-shrink: 0;
  text-align: center;
}
.sidebar-title {
  font-size: 1rem;
  font-weight: bold;
  line-height: 1.4;
  letter-spacing: 0.03em;
  padding-bottom: 0.75rem;
  border-bottom: 1px solid var(--border);
}
.sidebar-nav { padding: 0.75rem 0; flex: 1; }
.nav-group-label {
  padding: 0.75rem 1rem 0.25rem;
  font-size: 0.75rem;
  letter-spacing: 0.15em;
  color: var(--muted-soft);
  text-transform: uppercase;
  font-weight: bold;
}
.nav-item {
  display: block;
  width: 100%;
  min-height: 2.75rem;
  padding: 0.5rem 1rem;
  background: none;
  border: none;
  text-align: left;
  font-size: 0.9375rem;
  line-height: 1.45;
  cursor: pointer;
  color: var(--text);
}
.nav-item:hover { background: var(--nav-hover); }
.nav-item.active {
  background: var(--nav-active-bg);
  color: var(--nav-active-text);
  font-weight: bold;
}
.main-content {
  flex: 1;
  min-height: 0;
  overflow-y: auto;
  display: flex;
  justify-content: center;
}
.page-container {
  width: 100%;
  max-width: calc(var(--content-max) + 2rem);
  background: var(--surface);
  min-height: 100%;
  padding: 1.5rem 1rem 3rem;
  box-shadow: 0 0 20px rgba(0, 0, 0, 0.03);
}
.page-view { display: none; }
.page-view.active { display: block; }
.section-header {
  margin-bottom: 1.875rem;
  padding-bottom: 1.25rem;
  border-bottom: 2px solid var(--text);
  text-align: center;
}
.section-tag {
  font-size: 0.75rem;
  letter-spacing: 0.15em;
  color: var(--muted);
  text-transform: uppercase;
  margin-bottom: 0.5rem;
}
.section-header h2 {
  margin: 0.625rem 0;
  font-size: clamp(1.75rem, 4.5vw, 2.25rem);
  font-weight: normal;
  line-height: 1.3;
}
.subtitle {
  margin: 0;
  color: var(--muted);
  font-size: 1rem;
  font-style: italic;
}
.meta-row {
  display: flex;
  flex-wrap: wrap;
  gap: 0.75rem;
  margin-top: 1.25rem;
  justify-content: center;
}
.pill {
  display: inline-flex;
  align-items: center;
  padding: 0.25rem 0.75rem;
  font-size: 0.875rem;
  color: var(--muted);
  font-style: italic;
}
.block-title {
  margin: 2rem 0 1rem;
  font-size: 1.125rem;
  font-weight: bold;
  text-align: center;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}
.plot-card {
  margin: 0 auto 1rem;
  width: 100%;
  max-width: var(--content-max);
  display: flex;
  justify-content: center;
}
.plot-card .js-plotly-plot, .plot-card .plotly-graph-div {
  width: 100% !important;
  margin: 0 auto;
}
.plotly .modebar, .modebar { display: none !important; }
.note {
  text-align: center;
  margin: 0.75rem auto 1.5rem;
  color: var(--muted);
  font-size: 0.875rem;
  font-style: italic;
}
.analysis-card, .gallery-card, .gallery-ref, .reader-section {
  border: 1px solid var(--border);
  background: #fffdfa;
  border-radius: 0.375rem;
  max-width: var(--content-max);
  margin: 0.75rem auto 1.125rem;
  padding: 0.875rem 1rem;
}
.analysis-title, .gallery-title {
  font-size: 0.9375rem;
  font-weight: bold;
  letter-spacing: 0.03em;
  margin-bottom: 0.5rem;
}
.analysis-list {
  margin: 0;
  padding-left: 1.125rem;
  line-height: 1.6;
}
.reader-section-title {
  margin: 0 0 0.75rem;
  font-size: 1.125rem;
  font-weight: bold;
}
.section-copy,
.table-note {
  margin: 0 0 1rem;
  font-size: 0.95rem;
  line-height: 1.7;
  color: var(--text);
}
.reader-table-title {
  margin: 1rem 0 0.5rem;
  font-size: 0.95rem;
  font-weight: bold;
  color: var(--text);
}
code {
  word-break: break-all;
  background: rgba(0, 0, 0, 0.04);
  padding: 0 4px;
  border-radius: 3px;
}
.table-card {
  margin: 1rem auto 2rem;
  width: 100%;
  max-width: var(--table-max);
  overflow-x: auto;
  display: flex;
  justify-content: center;
}
.metrics-table { width: 100%; border-collapse: collapse; text-align: center; }
.metrics-table th, .metrics-table td { padding: 0.625rem 0.75rem; font-size: 0.875rem; line-height: 1.55; border: none; vertical-align: top; }
.metrics-table thead tr:first-child th { border-top: 1.5px solid var(--text); border-bottom: 1px solid var(--text); }
.metrics-table tbody tr:last-child td { border-bottom: 1.5px solid var(--text); }
.metrics-table th:first-child, .metrics-table td:first-child { text-align: left; }
.intro-table-card {
  max-width: var(--content-max);
  overflow: visible;
  display: block;
}
.intro-table {
  width: 100%;
  table-layout: fixed;
}
.intro-table th,
.intro-table td {
  text-align: left;
  white-space: normal;
  overflow-wrap: anywhere;
  word-break: break-word;
  padding: 0.5rem 0.625rem;
  font-size: 0.8125rem;
  line-height: 1.5;
}
.intro-table--dataset-diff th:nth-child(1), .intro-table--dataset-diff td:nth-child(1) { width: 8%; }
.intro-table--dataset-diff th:nth-child(2), .intro-table--dataset-diff td:nth-child(2) { width: 11%; }
.intro-table--dataset-diff th:nth-child(3), .intro-table--dataset-diff td:nth-child(3) { width: 11%; }
.intro-table--dataset-diff th:nth-child(4), .intro-table--dataset-diff td:nth-child(4) { width: 10%; }
.intro-table--dataset-diff th:nth-child(5), .intro-table--dataset-diff td:nth-child(5) { width: 10%; }
.intro-table--dataset-diff th:nth-child(6), .intro-table--dataset-diff td:nth-child(6) { width: 12%; }
.intro-table--dataset-diff th:nth-child(7), .intro-table--dataset-diff td:nth-child(7) { width: 38%; }
.intro-table--experiment-design th:nth-child(1), .intro-table--experiment-design td:nth-child(1) { width: 18%; }
.intro-table--experiment-design th:nth-child(2), .intro-table--experiment-design td:nth-child(2) { width: 23%; }
.intro-table--experiment-design th:nth-child(3), .intro-table--experiment-design td:nth-child(3) { width: 24%; }
.intro-table--experiment-design th:nth-child(4), .intro-table--experiment-design td:nth-child(4) { width: 35%; }
.intro-table--state-config th:nth-child(1), .intro-table--state-config td:nth-child(1) { width: 12%; }
.intro-table--state-config th:nth-child(2), .intro-table--state-config td:nth-child(2) { width: 7%; }
.intro-table--state-config th:nth-child(3), .intro-table--state-config td:nth-child(3) { width: 17%; }
.intro-table--state-config th:nth-child(4), .intro-table--state-config td:nth-child(4) { width: 17%; }
.intro-table--state-config th:nth-child(5), .intro-table--state-config td:nth-child(5) { width: 21%; }
.intro-table--state-config th:nth-child(6), .intro-table--state-config td:nth-child(6) { width: 26%; }
.intro-table--observation-config th:nth-child(1), .intro-table--observation-config td:nth-child(1) { width: 11%; }
.intro-table--observation-config th:nth-child(2), .intro-table--observation-config td:nth-child(2) { width: 14%; }
.intro-table--observation-config th:nth-child(3), .intro-table--observation-config td:nth-child(3) { width: 18%; }
.intro-table--observation-config th:nth-child(4), .intro-table--observation-config td:nth-child(4) { width: 15%; }
.intro-table--observation-config th:nth-child(5), .intro-table--observation-config td:nth-child(5) { width: 22%; }
.intro-table--observation-config th:nth-child(6), .intro-table--observation-config td:nth-child(6) { width: 20%; }
.module-divider { border: 0; border-top: 1px dashed var(--border); margin: 3.5rem auto; width: 86%; max-width: var(--content-max); }
.gallery-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(16rem, 1fr));
  gap: 0.875rem;
}
.gallery-item {
  margin: 0;
  border: 1px solid var(--border);
  border-radius: 0.25rem;
  overflow: hidden;
  background: #ffffff;
}
.gallery-item img {
  display: block;
  width: 100%;
  height: auto;
}
.gallery-item figcaption {
  padding: 0.5rem 0.625rem;
  font-size: 0.8125rem;
  color: var(--muted);
  text-align: center;
}
.gallery-ref {
  font-size: 0.875rem;
  color: var(--muted);
}
.gallery-ref a { color: #0a5b8c; }
@media (min-width: 768px) {
  body { flex-direction: row; }
  .sidebar {
    position: fixed;
    left: 0;
    width: var(--sidebar-w);
    height: 100vh;
    max-height: none;
    border-right: 1px solid var(--border);
    border-bottom: none;
  }
  .main-content {
    margin-left: var(--sidebar-w);
    min-height: 100vh;
  }
  .page-container {
    max-width: calc(var(--content-max) + 4rem);
    padding: 2.5rem 2rem 4rem;
  }
}
@media (min-width: 1024px) {
  .page-container {
    padding: 2.5rem 3.75rem 5rem;
  }
}
"""

    page_ids = [page.page_id for page in report.pages]
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
  var main = document.querySelector('.main-content');
  if (main) main.scrollTop = 0;
}
document.addEventListener('DOMContentLoaded', function() {
  showPage('""" + page_ids[0] + """');
});
"""

    parts: list[str] = [
        "<!DOCTYPE html>",
        '<html lang="zh-CN">',
        "<head>",
        '<meta charset="utf-8" />',
        '<meta name="viewport" content="width=device-width, initial-scale=1" />',
        f"<title>{escape(report.report_title)}</title>",
        _plotly_tag(out_path, inline_assets=inline_assets),
        f"<style>{css}</style>",
        "</head>",
        "<body>",
        '<aside class="sidebar">',
        '<div class="sidebar-header">',
        f'<div class="sidebar-title">{escape(report.report_title)}</div>',
        "</div>",
        '<nav class="sidebar-nav">',
    ]

    group_title_map = {group.group_id: group.title for group in report.groups}
    for group in report.groups:
        parts.append(f'<div class="nav-group-label">{escape(group.title)}</div>')
        for page in [p for p in report.pages if p.group_id == group.group_id]:
            parts.append(
                f'<button class="nav-item" data-page="{escape(page.page_id)}" onclick="showPage(\'{escape(page.page_id)}\')">'
                f"{escape(page.nav_label)}"
                "</button>"
            )

    parts.extend(["</nav>", "</aside>", '<div class="main-content">', '<div class="page-container">'])

    seen_galleries: dict[tuple[str, str], str] = {}
    gallery_asset_dirs: dict[str, Path] = {}
    for page in report.pages:
        parts.extend(
            [
                f'<div class="page-view" id="{escape(page.page_id)}">',
                '<div class="section-header">',
                f'<div class="section-tag">{escape(group_title_map.get(page.group_id, page.group_id))}</div>',
                f"<h2>{escape(page.title)}</h2>",
                f'<p class="subtitle">{escape(page.subtitle)}</p>',
            ]
        )
        if page.section_result is not None:
            section = page.section_result
            parts.append('<div class="meta-row">')
            if section.split_time_s is not None:
                parts.append(f'<span class="pill">GNSS 截止时刻: {section.split_time_s:.3f} s</span>')
            parts.append(f'<span class="pill">实验组数量: {len(section.cases)}</span>')
            parts.append("</div>")
        parts.append("</div>")

        if page.intro_sections:
            parts.append(_render_intro_sections(page))

        if page.section_result is not None:
            section = page.section_result
            parts.extend(['<h3 class="block-title">轨迹图</h3>', '<div class="plot-card">'])
            parts.append(
                section.trajectory_fig.to_html(
                    full_html=False,
                    include_plotlyjs=False,
                    config=PLOT_CONFIG,
                )
            )
            parts.append("</div>")
            parts.append('<h3 class="block-title">总览指标</h3>')
            parts.append(table_block(section.overview_table, decimals=3, rename_map=OVERVIEW_TABLE_RENAME))
            parts.append(_section_timing_note(page))
            parts.append('<hr class="module-divider" />')

            for idx, (category_key, category_title, _) in enumerate(CATEGORY_SPECS):
                parts.append(f'<h3 class="block-title">{escape(category_title)}</h3>')
                parts.append('<div class="plot-card">')
                parts.append(
                    section.figures[category_key].to_html(
                        full_html=False,
                        include_plotlyjs=False,
                        config=PLOT_CONFIG,
                    )
                )
                parts.append("</div>")
                parts.append(table_block(section.stats_tables[category_key], rename_map=STATS_TABLE_RENAME))
                if idx < len(CATEGORY_SPECS) - 1:
                    parts.append('<hr class="module-divider" />')

        if page.analysis_html:
            parts.append(page.analysis_html)
        state_html = _render_state_section(
            page,
            out_path,
            seen_galleries,
            gallery_asset_dirs,
            inline_assets=inline_assets,
        )
        if state_html:
            parts.append(state_html)
        parts.append("</div>")

    parts.extend(["</div>", "</div>", f"<script>{js}</script>", "</body>", "</html>"])
    out_path.write_text("\n".join(parts), encoding="utf-8")
    return out_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export the dataset-partitioned navigation HTML report.")
    parser.add_argument("--target-points", type=int, default=2500, help="Maximum points per trace after decimation.")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("output/html/dataset_partitioned_navigation_interactive_report.html"),
        help="Output HTML file path.",
    )
    parser.add_argument(
        "--asset-mode",
        choices=("inline", "external"),
        default="inline",
        help="How to package Plotly and gallery images. 'inline' creates a self-contained distributable HTML.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_path = render_html(
        target_points=args.target_points,
        out_path=args.out,
        inline_assets=args.asset_mode == "inline",
    )
    print(out_path.resolve())


if __name__ == "__main__":
    main()
