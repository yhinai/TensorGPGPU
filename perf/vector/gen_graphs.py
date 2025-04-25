import argparse
import matplotlib.pyplot as plt
import re
import numpy as np
import pandas as pd
from itertools import cycle

def main():
    parser = argparse.ArgumentParser(description="Plot grouped benchmark comparison.")
    parser.add_argument('--benchmarks', nargs='+', required=True,
                       help='Benchmark names to compare')
    parser.add_argument('--designs', nargs='+', required=True,
                       help='Design names to compare (e.g., X Y)')
    parser.add_argument('--logfiles', nargs='+', required=True,
                       help='Paths to log files in order: [X_A, Y_A, X_B, Y_B, ...]')
    parser.add_argument('--counter', required=True,
                       help='Performance counter to extract')
    parser.add_argument('--ylabel', required=True,
                       help='Y-axis label for the performance counter')
    parser.add_argument('--title', required=True,
                       help='Title for the graph')
    parser.add_argument('--xlabel', required=True,
                       help='X-axis label for benchmark groups')
    parser.add_argument('--output', required=True,
                       help="Output file path of Excel table")
    parser.add_argument('--plot', required=True,
                       help="Output file path of Figures")
    parser.add_argument('--group_size', type=int,
                       help="Group benchmarks into categories of this size")
    parser.add_argument('--group_spacing', type=float, default=1.0,
                       help="Spacing between benchmark groups (default: 1.0)")
    parser.add_argument('--bar_width', type=float, default=0.8,
                       help="Width of each bar (default: 0.8)")

    args = parser.parse_args()

    # Validate inputs
    num_benchmarks = len(args.benchmarks)
    num_designs = len(args.designs)
    expected_files = num_benchmarks * num_designs
    if len(args.logfiles) != expected_files:
        raise ValueError(f"Expected {expected_files} log files ({num_benchmarks} benchmarks × {num_designs} designs)")

    # Extract data from log files (new ordering: X_A, Y_A, X_B, Y_B, ...)
    values = np.empty((num_designs, num_benchmarks))
    for bench_idx in range(num_benchmarks):
        for design_idx in range(num_designs):
            file_idx = bench_idx * num_designs + design_idx
            with open(args.logfiles[file_idx], 'r') as file:
                content = file.read()
                match = re.findall(args.counter + r'=(\d+(?:\.\d+)?)', content)
                if not match:
                    raise ValueError(f"Counter '{args.counter}' not found in {args.logfiles[file_idx]}")
                values[design_idx][bench_idx] = float(match[-1])

    # Save data to Excel
    data = {'Benchmarks': args.benchmarks}
    for i, design in enumerate(args.designs):
        data[design] = values[i]
    df = pd.DataFrame(data)
    df.to_excel(args.output, index=False, engine='openpyxl')

    # Plotting setup
    fig, ax = plt.subplots(figsize=(10, 6))

    # Calculate positions and widths
    if args.group_size:
        num_groups = (num_benchmarks + args.group_size - 1) // args.group_size
        group_width = args.bar_width / num_designs
        group_centers = np.arange(num_groups) * (args.group_size + args.group_spacing)

        # Calculate positions for each design within each group
        positions = []
        for group_idx in range(num_groups):
            for design_idx in range(num_designs):
                positions.append(group_centers[group_idx] + design_idx * group_width)
    else:
        # Non-grouped mode
        group_width = args.bar_width / num_designs
        group_centers = np.arange(num_benchmarks) * (1 + args.group_spacing)
        positions = []
        for bench_idx in range(num_benchmarks):
            for design_idx in range(num_designs):
                positions.append(group_centers[bench_idx] + design_idx * group_width)

    # Create colors for designs
    colors = cycle(plt.rcParams['axes.prop_cycle'].by_key()['color'])
    design_colors = {design: next(colors) for design in args.designs}

    # Plot bars
    all_bars = []
    for design_idx, design in enumerate(args.designs):
        if args.group_size:
            # For grouped data, we need to aggregate values within each group
            grouped_values = []
            for group_idx in range(num_groups):
                start = group_idx * args.group_size
                end = start + args.group_size
                group_vals = values[design_idx][start:end]
                grouped_values.append(np.mean(group_vals))  # or sum() depending on needs
            bars = ax.bar(
                [group_centers[g] + design_idx * group_width for g in range(num_groups)],
                grouped_values,
                width=group_width,
                label=design,
                color=design_colors[design]
            )
        else:
            bars = ax.bar(
                [group_centers[b] + design_idx * group_width for b in range(num_benchmarks)],
                values[design_idx],
                width=group_width,
                label=design,
                color=design_colors[design]
            )
        all_bars.extend(bars)

    # Customize plot
    if args.group_size:
        # Grouped mode
        ax.set_xticks(group_centers + (num_designs - 1) * group_width / 2)
        ax.set_xticklabels([f"Group {i+1}" for i in range(num_groups)])

        # Add benchmark names as x-axis minor ticks
        minor_pos = []
        minor_labels = []
        for group_idx in range(num_groups):
            start = group_idx * args.group_size
            end = min(start + args.group_size, num_benchmarks)
            for bench_idx in range(start, end):
                minor_pos.append(group_centers[group_idx] +
                               (bench_idx % args.group_size) * (group_width * num_designs) / args.group_size)
                minor_labels.append(args.benchmarks[bench_idx])
        ax.set_xticks(minor_pos, minor=True)
        ax.set_xticklabels(minor_labels, minor=True)
        ax.tick_params(axis='x', which='minor', length=0)  # Hide minor tick marks
    else:
        # Non-grouped mode
        ax.set_xticks(group_centers + (num_designs - 1) * group_width / 2)
        ax.set_xticklabels(args.benchmarks)

    ax.set_xlabel(args.xlabel)
    ax.set_ylabel(args.ylabel)
    ax.set_title(args.title)
    ax.legend(title="Designs")

    # Adjust x-axis limits to include spacing at both ends
    if args.group_size:
        x_min = -0.5
        x_max = group_centers[-1] + args.group_size + 0.5
    else:
        x_min = -0.5
        x_max = group_centers[-1] + 1 + 0.5
    ax.set_xlim(x_min, x_max)

    plt.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(args.plot)
    plt.show()

if __name__ == "__main__":
    main()
