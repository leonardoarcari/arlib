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


# === ----------------------------------------------------------------------=== #
#                               Parser builders
# === ----------------------------------------------------------------------=== #


def get_programs_list():
    return [build_sim_time_pruning_parser]


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

# === ----------------------------------------------------------------------=== #
#                                Main function
# === ----------------------------------------------------------------------=== #


def main():
    # Parse arguments and dispatch
    args = parse_arguments()
    args.func(args)


if __name__ == "__main__":
    main()
