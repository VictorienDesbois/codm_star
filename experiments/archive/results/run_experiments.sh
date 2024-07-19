#!/usr/bin/bash

# parameters

time_limit=$1
memory_limit=$2

echo "[ time limit: $time_limit (s) | memory limit: $memory_limit (kB) ]"

# init directories

# 	logs

rm -rf logs_CASTAR
rm -rf plots/CASTAR
mkdir logs_CASTAR
mkdir plots/CASTAR

rm -rf logs_CODM
rm -rf plots/CODM
mkdir logs_CODM
mkdir plots/CODM

rm -rf logs_CODMOPTI
rm -rf plots/CODMOPTI
mkdir logs_CODMOPTI
mkdir plots/CODMOPTI

#   get pictures
rm -rf plots/pictures
mkdir plots/pictures

# run commands 

cmd2="./run_algorithms.sh run_algorithm CASTAR $time_limit $memory_limit"
cmd3="./run_algorithms.sh run_algorithm CODM $time_limit $memory_limit"
cmd4="./run_algorithms.sh run_algorithm CODMOPTI $time_limit $memory_limit"

$cmd2 &
$cmd3 &
$cmd4 &
wait