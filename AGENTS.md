# AGENT.md

This file defines how the coding agent must work in this repository.

## 1. Mission

Primary research goals:

- Compare different integrated navigation algorithm combinations.
- Study observability of state components under different sensor schedules and constraints.

Current baseline reference:

- Config: `config_data2_baseline_eskf.yaml`
- Core system: INS/GNSS/ODO/NHC integrated navigation
- State dimension: 31 (`kStateDim = 31`)
- Filter modes in code: standard ESKF and InEKF switch (`fusion.fej.enable`)

## 2. Session Lifecycle (Mandatory)

For every task, follow this order:

1. Start: open and read `walkthrough.md` first.
2. Continue: reuse active hypotheses, experiment IDs, and unresolved items from `walkthrough.md`.
3. Execute: run or edit according to the current task.
4. Close: update `walkthrough.md` before ending the task.

Do not skip steps 1 or 4.

## 3. Required Start Checklist

At task start, the agent must extract from `walkthrough.md`:

- Current phase objective.
- Last completed session entry.
- Open hypotheses (`HYP-*`).
- Pending next actions.
- Existing experiment naming context (`EXP-*`).

If `walkthrough.md` is missing, create it immediately using the bootstrap template below (do not defer work until a schema is found elsewhere).

Bootstrap template for a new `walkthrough.md`:

```markdown
# walkthrough.md

Schema: `v1`
Last updated: `YYYY-MM-DD`

## Project Snapshot
- long-term objective
- current baseline config
- state dimension and active filter modes

## Experiment Registry
| exp_id | date | purpose | config | key artifacts | key metrics | status | freshness |
|---|---|---|---|---|---|---|---|

## Known Inconsistencies
| issue_id | detected_on | description | affected files | impact | resolution status |
|---|---|---|---|---|---|

## Open Hypotheses
| hyp_id | hypothesis | evidence so far | next check | status |
|---|---|---|---|---|

## Session Log
### session_id: YYYYMMDD-HHMM-task_slug
- objective:
- scope:
- changed_files:
- configs:
- commands:
- artifacts:
- metrics:
- artifact_mtime:
- config_hash_or_mtime:
- dataset_time_window:
- result_freshness_check:
- observability_notes:
- decision:
- next_step:

## Next Actions
1. ...
```

## 4. Required End Checklist

Before ending any task, append one `Session Log` entry in `walkthrough.md` with:

- `session_id`
- `objective`
- `scope`
- `changed_files`
- `configs`
- `commands`
- `artifacts`
- `metrics`
- `observability_notes`
- `decision`
- `next_step`

Also update:

- `Experiment Registry` (for every new or updated experiment)
- `Open Hypotheses` (status change if validated/rejected)
- `Next Actions` (re-prioritized list)

## 5. Experiment ID and Naming Rules

Use deterministic IDs:

- Session: `YYYYMMDD-HHMM-task_slug`
- Experiment: `EXP-YYYYMMDD-short_name`
- Hypothesis: `HYP-<number>`

Artifact naming:

- Config files: `config_*`
- Solution output: `SOL_*`
- Logs: `output/*.log`
- Plots/summary: `output/*/`

Every experiment entry must include exact file paths.

## 6. Evidence Minimum for Any Conclusion

Do not record algorithm-performance conclusions without evidence. Each conclusion must reference:

- Config file path used.
- Command(s) executed.
- Output artifact path(s).
- Quantitative metric(s) (for example RMSE xyz, RMSE 3D, P95 3D, acceptance ratios when available).
- Freshness check (`artifact_mtime`, `result_freshness_check`).

If evidence conflicts with historical summary files, mark the entry as:

- `status: stale_or_conflict`

and record the conflict source.

## 7. Observability-Focused Logging Rules

When the task touches observability analysis, the session entry must map findings to state blocks:

- Extrinsics and scales: indices 21-30 as applicable.
- Ablation controls: `fusion.ablation.*`
- Post-GNSS freeze: `fusion.post_gnss_ablation.*`
- Schedule effects: `fusion.gnss_schedule.*`, `fusion.uwb_anchor_schedule.*`

Each observability note must include:

- Which state block was constrained/frozen/ablated.
- Which sensor update windows were active.
- Whether behavior improved, degraded, or remained neutral.

## 8. Core Pipeline Context to Keep Consistent

When writing entries, use the actual pipeline order in code:

1. Predict (IMU)
2. ZUPT
3. Gravity alignment diagnostics
4. NHC
5. ODO
6. UWB
7. GNSS (with optional schedule stop and post-GNSS ablation)
8. Diagnostics + result recording

## 9. Document Size Control

`walkthrough.md` update policy:

- Default: append one concise task-level summary per task.
- If `Session Log` grows beyond 20 entries or file size exceeds about 250 KB:
  - Compress old entries into a phase summary.
  - Keep all experiment IDs and decisions traceable.
  - Do not delete the latest unresolved items.

## 10. Scope and Safety Constraints

- Do not treat old metrics as valid unless their source artifacts and times are verified.
- Do not overwrite previous conclusions silently; add superseding notes with explicit reason.
- Keep all path references local and reproducible from repository root.
- Prefer concise records over full raw logs in `walkthrough.md`; store long logs in `output/` and link paths.

## 11. Context Exhaustion Emergency Protocol (Mandatory)

If the context window is close to exhaustion, do not continue normal execution.

Required emergency action:

1. Immediately execute the End Checklist, even if the task is not complete.
2. Append a `Session Log` entry with `status: interrupted_due_to_context_limit`.
3. In `next_step`, record an explicit resume checkpoint including:
   - exact file path and line/function where work stopped
   - last command executed (or last planned command if not executed)
   - unresolved blockers/assumptions
   - exact next command or edit to run first on resume
4. Prioritize preserving reproducibility over completeness.
