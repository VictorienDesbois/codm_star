#!/usr/bin/bash

# parameters

time_limit=$1
memory_limit=$2
swapping_conflicts=$3

echo "[ time limit: $time_limit (s) | memory limit: $memory_limit (kB) ]"

# init directories

# 	logs

if [ $swapping_conflicts == "true" ]
then
	cd swapping

	# rm -rf logs_CASTAR_SWAPP
	# rm -rf plots/CASTAR_SWAPP
	# mkdir logs_CASTAR_SWAPP
	# mkdir plots/CASTAR_SWAPP
else
	cd classic

	# rm -rf logs_CASTAR
	# rm -rf plots/CASTAR
	# mkdir logs_CASTAR
	# mkdir plots/CASTAR
fi

rm -rf logs_CODM
rm -rf plots/CODM
mkdir logs_CODM
mkdir plots/CODM

rm -rf logs_CODM-OPT
rm -rf plots/CODM-OPT
mkdir logs_CODM-OPT
mkdir plots/CODM-OPT

rm -rf logs_CODM-BIDIR
rm -rf plots/CODM-BIDIR
mkdir logs_CODM-BIDIR
mkdir plots/CODM-BIDIR

rm -rf logs_ODRM
rm -rf plots/ODRM
mkdir logs_ODRM
mkdir plots/ODRM

#   get pictures
rm -rf plots/pictures
mkdir plots/pictures

# run commands 

# if [ $swapping_conflicts ]
# then
	# cmd1="./run_algorithms.sh run_algorithm CASTAR $time_limit $memory_limit $swapping_conflicts"
# else
	# cmd1="./run_algorithms.sh run_algorithm CASTAR-SWAPP $time_limit $memory_limit $swapping_conflicts"
# fi

cmd2="./run_algorithms.sh run_algorithm CODM $time_limit $memory_limit $swapping_conflicts"
cmd3="./run_algorithms.sh run_algorithm CODM-OPT $time_limit $memory_limit $swapping_conflicts"
cmd4="./run_algorithms.sh run_algorithm CODM-BIDIR $time_limit $memory_limit $swapping_conflicts"
cmd5="./run_algorithms.sh run_algorithm ODRM $time_limit $memory_limit $swapping_conflicts"

# $cmd1 &
$cmd2 &
$cmd3 &
$cmd4 &
$cmd5 &
wait