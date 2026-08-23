# Exploration milestone - teammate artifact index

Use these two summary images first:

- `comparison/final_maps_comparison.png` - four-run visual board
- `comparison/coverage_comparison.png` - strategy comparison split by world

## Strategy summary

| Package | World | Strategy under test | Outcome |
|---|---|---|---|
| PR1 | `leo_world` | Hardened, planner-validated frontier exploration | PASS: 390.8 m2, about 99% of reachable area |
| PR2 | `husarion_office` | RPP + rotation shim controller | PASS/adopted: faster clean driving; exposed planner-reject livelock |
| PR3 | both worlds | Physical reverse/rotate escape from planner-reject livelock | PASS: real and injected failures both recovered |
| PR3 follow-up | `husarion_office` | Ghost-frontier long-ban (fixes ghost-grind livelock found in soak) | PASS: full office run terminates, returns home, saves map (`pr3/ghost_fix_run/`) |
| PR4 | `leo_world` | Camera-aware wall sweep and item confirmation | PASS: 90% wall-camera coverage, 6/6 items |

The worlds are intentionally different. `leo_world` is the repeatable coverage
and item-search benchmark. `husarion_office` contains non-physical gaps and is
used as an adversarial recovery/controller benchmark, so raw area values
should not be compared as if the maps had equal size.

## Videos

- `pr1/exploration.mp4` - hardened frontier exploration
- `pr2/exploration.mp4` - controller comparison and pathological office soak
- `pr3/office_escape.mp4` - natural real-planner escape
- `pr3/fault_injection_final/exploration.mp4` - deterministic rotate escape
- `pr3/ghost_fix_run/exploration.mp4` - full office run with the ghost-frontier
  long-ban fix: explores, sweeps, returns home, saves the map
- `pr3/ghost_grind_run/exploration_soak.mp4` - pre-fix soak evidence: the
  black-speckle ghost field behind the east wall and the coverage flatline
- `pr4/exploration.mp4` - full frontier exploration, camera sweep, and return

## Maps and evidence

- PR1/PR4 saved map bundles: PGM, YAML, pose graph, and serialized SLAM data
- PR2 office map: `pr2/office_map.png`
- PR3 raw acceptance logs and telemetry:
  `pr3/fault_injection_final/`
- PR4 pre-fix livelock evidence:
  `pr4/livelock_run/`

Each package directory contains its detailed `REPORT.md`.
