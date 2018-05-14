#!/bin/bash

# Per application definitions
CONF_NAME="kspwlo.conf"
APPLICATION_BIN="kspwlo"

# == --------------------------------------------------------------------- == #
# 								DSE Process						
# == --------------------------------------------------------------------- == #

function run_dse_for {
	local algorithm=$1
	local workspace=$TIMESTAMP/$algorithm

	echo "Running DSE for ${algorithm}..."

	# check if we already have a dse workspace to resume
	if [ -d $ROOT/dse/${APPLICATION_BIN}/$workspace ]; then
		echo "We already have a DSE workspace!"
		echo "Resuming the DSE!"

		# resume the DSE
		make -C $ROOT/dse/${APPLICATION_BIN}/$workspace || (echo "DSE interrupted! (you can resume it later)" && exit -1)

	else
		echo "Creating a DSE workspace!"

		# Compile the application in exploration mode
		/bin/bash $ROOT/build_exploration.sh || exit -1

		# Generate the workspace
		python3 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli generate_dse --workspace $ROOT/dse/${APPLICATION_BIN}/$workspace --executable $ROOT/build/$APPLICATION_BIN $ROOT/dse_apps/${algorithm}.xml && echo "OP list succesfully created!"

		# Resume the DSE
		make -C $ROOT/dse/${APPLICATION_BIN}/$workspace || (echo "DSE interrupted! (you can resume it later)" && exit -1)
	fi

	# Now plot data with GNU Plot
	GNUPLOT_SCRIPTS=("${algorithm}_sim-resp_time-pruning.gnuplot"
					"${algorithm}_tau-quality_and_resp_time-k.gnuplot")
	PDF_NAMES=("${algorithm}_sim-resp_time-pruning.pdf"
			"${algorithm}_tau-quality_and_resp_time-k.pdf")
	METRICS=("exec_time" "avg_total_distance"
			"avg_average_distance" "avg_decision_edges")

	for metric in "${METRICS[@]}"; do
		MARGOT_GNUPLOT="${algorithm}_${metric}.gnuplot"
		MARGOT_PDF="${algorithm}_${metric}.pdf"
		# Margot Heel plot
		python2.7 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli plotOPs --x_field k_paths --y_field similarity_threshold --c_field $metric $ROOT/dse/${APPLICATION_BIN}/$workspace/oplist.conf > $ROOT/dse/${APPLICATION_BIN}/$workspace/$MARGOT_GNUPLOT && echo "GNUPLOT script succesfully created!"
		(gnuplot < $ROOT/dse/${APPLICATION_BIN}/$workspace/$MARGOT_GNUPLOT > $ROOT/dse/${APPLICATION_BIN}/$workspace/$MARGOT_PDF) || exit -1
	done


	# Thesis plot
	OPLIST_CSV=/tmp/oplist.csv
	python3 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli xml2csv $ROOT/dse/${APPLICATION_BIN}/$workspace/oplist.conf > $OPLIST_CSV

	python3 $ROOT/thesis_plotter.py sim_time_pruning --k 5 --csv-file $OPLIST_CSV $ROOT/dse/${APPLICATION_BIN}/$workspace/${algorithm}_sim-resp_time-pruning.gnuplot  && echo "GNUPLOT Sim|RespTime|Pruning script succesfully created!"
	python3 $ROOT/thesis_plotter.py pruning_quality_and_time_k --sim 0.75 --csv-file $OPLIST_CSV $ROOT/dse/${APPLICATION_BIN}/$workspace/${algorithm}_tau-quality_and_resp_time-k.gnuplot  && echo "GNUPLOT Tau|Quality&RespTime|K script succesfully created!"

	NB_PLOTS=${#GNUPLOT_SCRIPTS[@]}
	for ((i=0; i<${NB_PLOTS}; i++)); do
		(gnuplot < $ROOT/dse/${APPLICATION_BIN}/$workspace/${GNUPLOT_SCRIPTS[$i]} > $ROOT/dse/${APPLICATION_BIN}/$workspace/${PDF_NAMES[$i]}) || exit -1
	done

	echo "DSE for ${algorithm} done!"
}

# == --------------------------------------------------------------------- == #

# HELP
HELP="Usage: $0 [opplus|esx|penalty|all]"
# Check number of arguments
if [ $# -ne 1 ]; then
	echo "Wrong number of parameters: $@. Required 2."
	echo $HELP
	exit -1
fi

# Parse algorithm name and set WORKSPACE
ALGORITHMS=("opplus" "esx" "penalty")
OPT=$1
JOBS=()
for alg in "${ALGORITHMS[@]}"; do
	if [ "$OPT" = "$alg" ]; then
		JOBS=("$alg")
		shift
		break
	elif [ "$OPT" = "all" ]; then
		JOBS=("opplus" "esx" "penalty")
		shift
		break
	fi
done
# If unknown algorithm:
if [ "${#JOBS[@]}" -eq 0 ]; then
	echo "Unknown parameter \"$OPT\"."
	echo $HELP
	exit -1
fi

# Compute paths
ROOT=`git rev-parse --show-toplevel`
MARGOT_ROOT=$ROOT/../core2/
TIMESTAMP=`date "+%Y%m%d"`

for alg in "${JOBS[@]}"; do
	run_dse_for "$alg"
done

