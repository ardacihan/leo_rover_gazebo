#!/usr/bin/env python3
"""Render a presentation-ready coordinated/independent comparison."""

import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from phase2_metrics import (  # noqa: E402
    coverage_series,
    duplicated_area,
    evaluation_coverage_series,
)


COLORS = {"coordinated": "#2a9d8f", "independent": "#e76f51"}


def aligned_series(run_dir, world, source):
    series = (
        coverage_series(os.path.join(run_dir, "coverage.log"))
        if source == "runtime"
        else evaluation_coverage_series(run_dir, world)
    )
    if not series:
        return [], []
    start = series[0][0]
    return [(t - start) / 60.0 for t, _ in series], [v for _, v in series]


def value_at(times, values, horizon):
    pairs = [(t, v) for t, v in zip(times, values) if t <= horizon]
    return pairs[-1][1] if pairs else float("nan")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("world")
    parser.add_argument("coordinated")
    parser.add_argument("independent")
    parser.add_argument("output")
    parser.add_argument(
        "--coverage-source",
        choices=("auto", "runtime"),
        default="auto",
        help="Use runtime for legacy runs whose local maps were world-framed.",
    )
    args = parser.parse_args()

    runs = {
        "coordinated": args.coordinated,
        "independent": args.independent,
    }
    curves = {
        name: aligned_series(path, args.world, args.coverage_source)
        for name, path in runs.items()
    }
    horizon = min(times[-1] for times, _ in curves.values() if times)
    area = {
        name: value_at(times, values, horizon)
        for name, (times, values) in curves.items()
    }
    duplicate = {
        name: duplicated_area(path, args.world) for name, path in runs.items()
    }

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
    # Fixed margins are more reliable than tight/constrained layout here:
    # long suptitles plus a two-line right title can otherwise push the left
    # axes outside the saved canvas on different matplotlib releases.
    fig.subplots_adjust(
        left=0.07, right=0.98, bottom=0.13, top=0.76, wspace=0.14
    )
    for name, (times, values) in curves.items():
        axes[0].plot(
            times,
            values,
            linewidth=2.5,
            color=COLORS[name],
            label=name.title(),
        )
    axes[0].axvline(horizon, color="#555", linestyle="--", linewidth=1)
    axes[0].set(
        title=f"{args.world.replace('_', ' ').title()}: mapped area",
        xlabel="Minutes since coverage recording began",
        ylabel="Known area [m2]",
    )
    axes[0].grid(alpha=0.25)
    axes[0].legend(frameon=False)

    labels = ["Coordinated", "Independent"]
    names = ["coordinated", "independent"]
    bars = axes[1].bar(
        labels,
        [duplicate[name] for name in names],
        color=[COLORS[name] for name in names],
    )
    for bar, name in zip(bars, names):
        axes[1].text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 1,
            f"{duplicate[name]:.1f}",
            ha="center",
            fontweight="bold",
        )
    duplicate_delta = duplicate["coordinated"] - duplicate["independent"]
    if duplicate["independent"] > 0 and duplicate_delta <= 0:
        reduction = 100.0 * (-duplicate_delta) / duplicate["independent"]
        duplicate_note = f"{reduction:.1f}% less"
    elif duplicate_delta > 0:
        duplicate_note = f"coordinated +{duplicate_delta:.1f} m2"
    else:
        duplicate_note = "no overlap in either run"
    axes[1].set(
        title=f"Duplicated mapping area\n({duplicate_note})",
        ylabel="Area known by both rovers [m2]",
    )
    axes[1].grid(axis="y", alpha=0.25)
    fig.suptitle(
        f"Matched horizon {horizon:.1f} min: "
        f"coordinated {area['coordinated']:.1f} m2 vs "
        f"independent {area['independent']:.1f} m2",
        fontweight="bold",
        y=0.98,
    )
    fig.savefig(args.output, dpi=150)
    print(
        f"wrote {args.output}; horizon={horizon:.1f} min, "
        f"area={area}, duplicate={duplicate}"
    )


if __name__ == "__main__":
    main()
