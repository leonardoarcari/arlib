import argparse
import sys
import csv

# === ----------------------------------------------------------------------=== #
#                               GNUPLOT generators
# === ----------------------------------------------------------------------=== #


def sim_time_pruning(args):
    # Parse csv and extract data to plot
    data = {}
    data_fmt = '{}\t{}\n'
    tau = 0

    with open(args.csv_file, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=',')
        for row in reader:
            if int(row['#k_paths']) != args.k:
                continue
            sim = float(row['#similarity_threshold'])
            time = float(row['exec_time'])
            tau = float(row['#prune'])
            d_str = data.setdefault(tau, '')
            d_str += data_fmt.format(sim, time)
            data[tau] = d_str

    with open(vars(args)['gnuplot-file'], 'w') as gnuplot:
        gnuplot.write(GNUPLOT_SIM_TIME_PRUNING)
        gnuplot.write('\n')

        plot = 'plot '
        for i, k in enumerate(sorted(data.keys())):
            if k < 0:
                title = 'No pruning'
            else:
                title = 'UBP (tau = {})'.format(k)

            plot += '\'-\' u 1:2 w points ls {} title "{}", \\\n'.format(
                i + 1, title)
        plot = plot[:-4]
        gnuplot.write(plot)
        gnuplot.write('\n\n# --- Data points\n')

        for k in sorted(data.keys()):
            gnuplot.write(data[k])
            gnuplot.write('e\n')


def pruning_quality_and_time_k(args):
    # Parse csv and extract data to plot
    data = {}
    data_fmt = '{}\t{}\n'
    k = 0

    with open(args.csv_file, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=',')
        for row in reader:
            sim_tollerance = abs(
                float(row['#similarity_threshold']) - args.sim)
            if sim_tollerance < 1e-08:
                k = int(row['#k_paths'])
                if k == 1:
                    continue
                tau = float(row['#prune'])
                if tau < 0:
                    tau = 1.4

                time = float(row['exec_time'])
                total_distance = float(row['avg_total_distance'])
                avg_distance = float(row['avg_average_distance'])
                quality = total_distance - avg_distance + 1
                d_str = data.setdefault(k, [{}, {}])
                d_str[0][tau] = quality
                d_str[1][tau] = time
                data[k] = d_str

    with open(vars(args)['gnuplot-file'], 'w') as gnuplot:
        gnuplot.write(GNUPLOT_TAU_QUALITY_AND_TIME_K)
        plot_title = ('set multiplot layout 2, 1 title '
                      '"Quality vs. Response time tradeoff '
                      '[{{/Symbol q}} = {}]"\n')
        gnuplot.write(plot_title.format(args.sim))
        gnuplot.write(
            'set ylabel "(totalDistance - averageDistance + 1)" font ", 20"\n')
        gnuplot.write('\n')

        plot = 'plot '
        for i, k in enumerate(sorted(data.keys())):
            title = "k = {}".format(k)

            plot += '\'-\' u 1:2 w linespoints ls {} title "{}", \\\n'.format(
                i + 1, title)
        plot = plot[:-4]
        gnuplot.write(plot)
        gnuplot.write('\n# --- Data points\n')

        for k in sorted(data.keys()):
            for tau in sorted(data[k][0].keys()):
                quality = data[k][0][tau]
                gnuplot.write(data_fmt.format(tau, quality))
            gnuplot.write('e\n')

        TIME_PLOT = ('unset key\n'
                     'set ylabel "Response time [ms]" font ", 37"\n'
                     'set xlabel "Uninformed Bidirectional Pruning '
                     '[{{/Symbol q}}]" font ", 37"\n\n')
        gnuplot.write(TIME_PLOT)
        gnuplot.write(plot)
        gnuplot.write('\n# --- Data points\n')
        for k in sorted(data.keys()):
            for tau in sorted(data[k][1].keys()):
                time = data[k][1][tau]
                gnuplot.write(data_fmt.format(tau, time))
            gnuplot.write('e\n')

# === ----------------------------------------------------------------------=== #
#                               Parser builders
# === ----------------------------------------------------------------------=== #


def get_programs_list():
    return [build_sim_time_pruning_parser, build_pruning_quality_and_time_k_parser]


def build_sim_time_pruning_parser(subparsers):
    p = subparsers.add_parser(
        'sim_time_pruning',
        help='Plots the "sim. vs exec. time vs pruning factor" graph')
    p.add_argument('--k', type=int, help='The number of alternative paths')
    p.add_argument('--csv-file', type=str, metavar='CSV',
                   help='Operating Point list in csv format')
    p.add_argument('gnuplot-file', type=str, metavar='GNUPLOT',
                   help='The output gnuplot file')
    p.set_defaults(func=sim_time_pruning)


def build_pruning_quality_and_time_k_parser(subparsers):
    p = subparsers.add_parser(
        'pruning_quality_and_time_k',
        help='Plots the "sim. vs quality & exec. time vs k_paths" graph')
    p.add_argument('--sim', type=float, help='The similarity threshold')
    p.add_argument('--csv-file', type=str, metavar='CSV',
                   help='Operating Point list in csv format')
    p.add_argument('gnuplot-file', type=str, metavar='GNUPLOT',
                   help='The output gnuplot file')
    p.set_defaults(func=pruning_quality_and_time_k)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='A GNUPlot script generator for my thesis.  --Leonardo Arcari')
    subparsers = parser.add_subparsers(help='Available plotters')
    programs = get_programs_list()

    # Add all the subcommands
    for add_to in programs:
        add_to(subparsers)

    # Parse args and return
    args = parser.parse_args()
    return args


# === ----------------------------------------------------------------------=== #
#                               GNUPLOT preambles
# === ----------------------------------------------------------------------=== #

GNUPLOT_SIM_TIME_PRUNING = ('reset\n'
                            'set terminal pdf size 20, 10 enhanced font ",50"\n'
                            'set style line 11 lc rgb \'#000000\' lt 1\n'
                            'set border back 3 ls 11\n'
                            'set tics nomirror in\n'
                            'set style line 12 lc rgb \'#000000\' lt 0 lw 2\n'
                            'set grid xtics ytics ls 12\n'
                            'set key above horizontal\n'
                            'set style line 1 lc rgb \'#1B9E77\' pt 7 ps 6 lt 1 lw 4  # --- dark teal\n'
                            'set style line 2 lc rgb \'#D95F02\' pt 7 ps 2 lt 1 lw 4  # --- dark orange\n'
                            'set style line 3 lc rgb \'#7570B3\' pt 7 ps 2 lt 1 lw 4  # --- dark lilac\n'
                            'set style line 4 lc rgb \'#E7298A\' pt 7 ps 2 lt 1 lw 4  # --- dark magenta\n'
                            'set xlabel "Similarity threshold [%]"\n'
                            'set ylabel "Response time [ms]"\n')

GNUPLOT_TAU_QUALITY_AND_TIME_K = ('reset\n'
                                  'set terminal pdf size 20, 10 enhanced font ",40"\n'
                                  'set style line 11 lc rgb \'#000000\' lt 1\n'
                                  'set border back 3 ls 11\n'
                                  'set tics nomirror in\n'
                                  'set style line 12 lc rgb \'#000000\' lt 0 lw 2\n'
                                  'set xtics out axis 1,0.1,1.4\n'
                                  'set xtics add ("No pruning" 1.4) \n'
                                  'set grid xtics ytics ls 12\n'
                                  'set key above horizontal\n'
                                  'set style line 1 lc rgb \'#1B9E77\' pt 7 ps 2 lt 1 lw 4  # --- dark teal\n'
                                  'set style line 2 lc rgb \'#D95F02\' pt 7 ps 2 lt 1 lw 4  # --- dark orange\n'
                                  'set style line 3 lc rgb \'#7570B3\' pt 7 ps 2 lt 1 lw 4  # --- dark lilac\n'
                                  'set style line 4 lc rgb \'#E7298A\' pt 7 ps 2 lt 1 lw 4  # --- dark magenta\n')

# === ----------------------------------------------------------------------=== #
#                                Main function
# === ----------------------------------------------------------------------=== #


def main():
    # Parse arguments and dispatch
    args = parse_arguments()
    args.func(args)


if __name__ == "__main__":
    main()
