#!/bin/bash

# Per application definitions
CONF_NAME="kspwlo.conf"
APPLICATION_BIN="kspwlo"

# HELP
HELP="Usage: $0 [opplus|esx] [exec_time|avg_total_distance|avg_average_distance|avg_decision_edges]"
# Check number of arguments
if [ $# -ne 2 ]; then
	echo "Wrong number of parameters: $@. Required 2."
	echo $HELP
	exit -1
fi

# Parse algorithm name and set WORKSPACE
OPT=$1
if [ $OPT == "opplus" ]; then
	WORKSPACE="opplus"
	shift
elif [ $OPT == "esx" ]; then
	WORKSPACE="esx"
	shift
else
	echo "Unknown parameter \"$OPT\"."
	echo $HELP
	exit -1
fi

# Parse METRIC to plot
METRIC_OPT=$1
if [ $METRIC_OPT == "exec_time" ]; then
	METRIC=$METRIC_OPT
elif [ $METRIC_OPT == "avg_length" ]; then
	METRIC=$METRIC_OPT
elif [ $METRIC_OPT == "avg_total_distance" ]; then
	METRIC=$METRIC_OPT
elif [ $METRIC_OPT == "avg_average_distance" ]; then
	METRIC=$METRIC_OPT
elif [ $METRIC_OPT == "avg_decision_edges" ]; then
	METRIC=$METRIC_OPT
else
	echo "Unknown metric \"$METRIC_OPT\"."
	echo $HELP
	exit -1
fi

# Compute paths
WORKING_DIR=$PWD
APPLICATION_ROOT=$WORKING_DIR
MARGOT_ROOT=$WORKING_DIR/../core2/

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
	python3 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli generate_dse --workspace $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE --executable $APPLICATION_ROOT/build/$APPLICATION_BIN $APPLICATION_ROOT/dse_apps/${WORKSPACE}.xml && echo "OP list succesfully created!"

	# Resume the DSE
	make -C $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE || (echo "DSE interrupted! (you can resume it later)" && exit -1)
fi

# Now plot data with GNU Plot
python2.7 $MARGOT_ROOT/margot_heel/margot_heel_cli/bin/margot_cli plotOPs --x_field k_paths --y_field similarity_threshold --c_field $METRIC $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/oplist.conf > $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/${WORKSPACE}_gnuplot_script && echo "GNUPLOT script succesfully created!"

(gnuplot < $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/${WORKSPACE}_gnuplot_script > $APPLICATION_ROOT/dse/${APPLICATION_BIN}/$WORKSPACE/${WORKSPACE}_${METRIC}_gnuplot.pdf) || exit -1
