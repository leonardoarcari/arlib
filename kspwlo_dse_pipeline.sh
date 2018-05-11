#!/bin/bash

# Per application definitions
CONF_NAME="kspwlo.conf"
APPLICATION_BIN="kspwlo"

# HELP
HELP="Usage: $0 [opplus|esx|penalty]"
# Check number of arguments
if [ $# -ne 1 ]; then
	echo "Wrong number of parameters: $@. Required 2."
	echo $HELP
	exit -1
fi

# Parse algorithm name and set WORKSPACE
ALGORITHMS=("opplus" "esx" "penalty")
OPT=$1
ALGORITHM=""
for alg in "${ALGORITHMS[@]}"; do
	if [ "$OPT" = "$alg" ]; then
		ALGORITHM=$alg
		shift
		break
	fi
done
# If unknown algorithm:
if [ "$ALGORITHM" = "" ]; then
	echo "Unknown parameter \"$OPT\"."
	echo $HELP
	exit -1
fi

# Compute paths
WORKING_DIR=$PWD
APPLICATION_ROOT=$WORKING_DIR
MARGOT_ROOT=$WORKING_DIR/../core2/
TIMESTAMP=`date "+%Y%m%d"`
WORKSPACE=$TIMESTAMP/$ALGORITHM

# check if we already have a dse workspace to resume
if [ -d $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE ]; then
	echo "We already have a DSE workspace!"
	echo "Resuming the DSE!"

	# resume the DSE
	make -C $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE || (echo "DSE interrupted! (you can resume it later)" && exit -1)

else
	echo "Creating a DSE workspace!"

	# Compile the application in exploration mode
	/bin/bash $APPLICATION_ROOT/build_exploration.sh || exit -1

	# Generate the workspace
	python3 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli generate_dse --workspace $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE --executable $APPLICATION_ROOT/build/$APPLICATION_BIN $APPLICATION_ROOT/dse_apps/${ALGORITHM}.xml && echo "OP list succesfully created!"

	# Resume the DSE
	make -C $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE || (echo "DSE interrupted! (you can resume it later)" && exit -1)
fi

# Now plot data with GNU Plot
GNUPLOT_SCRIPTS=("${ALGORITHM}_sim-resp_time-pruning.gnuplot")
PDF_NAMES=("${ALGORITHM}_sim-resp_time-pruning.pdf")
METRICS=("exec_time" "avg_total_distance"
		 "avg_average_distance" "avg_decision_edges")

for metric in "${METRICS[@]}"; do
	MARGOT_GNUPLOT="${ALGORITHM}_${metric}.gnuplot"
	MARGOT_PDF="${ALGORITHM}_${metric}.pdf"
	# Margot Heel plot
	python2.7 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli plotOPs --x_field k_paths --y_field similarity_threshold --c_field $metric $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/oplist.conf > $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/$MARGOT_GNUPLOT && echo "GNUPLOT script succesfully created!"
	(gnuplot < $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/$MARGOT_GNUPLOT > $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/$MARGOT_PDF) || exit -1
done


# Thesis plot
OPLIST_CSV=/tmp/oplist.csv
python3 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli xml2csv $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/oplist.conf > $OPLIST_CSV
python3 $APPLICATION_ROOT/thesis_plotter.py sim_time_pruning --k 5 --csv-file $OPLIST_CSV $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/${ALGORITHM}_sim-resp_time-pruning.gnuplot  && echo "GNUPLOT Sim|RespTime|Pruning script succesfully created!"

NB_PLOTS=${#GNUPLOT_SCRIPTS[@]}
for ((i=0; i<${NB_PLOTS}; i++)); do
	(gnuplot < $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/${GNUPLOT_SCRIPTS[$i]} > $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/${PDF_NAMES[$i]}) || exit -1
done
