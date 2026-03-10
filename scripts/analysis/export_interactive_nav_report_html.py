from __future__ import annotations

import argparse
from html import escape
from pathlib import Path
import sys

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


def dataframe_html(df, decimals: int = 4) -> str:
    return df.round(decimals).to_html(index=False, border=0, classes="metrics-table", escape=False)


def table_block(df, decimals: int = 4) -> str:
    return f'<div class="table-card">{dataframe_html(df, decimals=decimals)}</div>'


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
            f"<li><strong>{escape(case.spec.label)}</strong>: "
            f"source {escape(str(source))}, "
            f"rpy [deg] = ({roll:.6f}, {pitch:.6f}, {yaw:.6f})</li>"
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


def analysis_block(section) -> str:
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
        '<div class="sidebar-eyebrow">Interactive Report</div>',
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
            f'<div class="section-tag">Experiment {idx:02d}</div>',
            f'<h2>{escape(section.spec.title)}</h2>',
            f'<p class="subtitle">{escape(section.spec.subtitle)}</p>',
            '<div class="meta-row">',
        ])
        if section.split_time_s is not None:
            parts.append(f'<span class="pill">GNSS cutoff: {section.split_time_s:.3f} s</span>')
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
        parts.append(table_block(section.overview_table, decimals=3))
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
            parts.append(table_block(section.stats_tables[category_key]))
            
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
