# Paper-ready figures (clean)

Regenerated from the same source data as the report plots, but with **no titles,
no suptitles, no annotation text-boxes, and no card chrome** — just axes, data,
legends and tight margins. Drop straight into LaTeX with `\includegraphics`.
Multi-panel report figures are split into individual panels so each is its own
subfigure. All at 200 dpi.

Regenerate any time with: `python scripts/make_paper_figures.py`

## single_robot/

| clean file | replaces (report_package plot) | contents |
|---|---|---|
| `verify_lidar_growth.png` | left panel of `verification_coverage_curves.png` | lidar known-area vs time, 3 worlds |
| `verify_camera_coverage.png` | right panel of `verification_coverage_curves.png` | camera wall coverage vs time, 3 worlds |
| `pr_coverage_leo_world.png` | left panel of `custom_vs_explorelite_coverage.png` | PR1 vs PR4 map growth (leo_world) |
| `pr_coverage_office_world.png` | right panel of `custom_vs_explorelite_coverage.png` | PR2 vs PR3 map growth (office) |
| `map_office_world.png`, `map_leo_world.png`, `map_depot_world.png`, `map_depot_aliased_slam.png` | the four cards in `verification_final_maps.png` / `custom_vs_explorelite_maps.png` | final occupancy maps (red trail, blue pose), already clean |

## two_robot/

| clean file | replaces | contents |
|---|---|---|
| `coverage_office_world.png`, `coverage_depot_world.png` | `{office,depot}_coverage_vs_time.png` | mapped area vs time: 1 / 2-uncoord / 2-coord |
| `separation_office_world.png`, `separation_depot_world.png` | `{office,depot}_rover_separation.png` | inter-rover distance vs time (mean in legend) |
| `maps_<world>_{single,independent,coordinated}.png` | panels of `{office,depot}_maps_and_trajectories.png` | one map+trajectory panel each (leo1 red, leo2 purple) |
| `finalmap_*.png` | `two_robot/final_maps/*.png` | merged final maps, already clean |

## Note on naming in the original package
`custom_vs_explorelite_coverage.png` / `_maps.png` in the report package actually
show the **PR1–PR4 strategy progression**, not a custom-vs-`explore_lite`
head-to-head (the README label was misleading). The true `explore_lite`
head-to-head render lives at `reports/engine_report/explore_lite_vs_custom.png`
and is *not* in the package — say the word and I'll add a clean version of it too.
