#!/usr/bin/bash

shell_script=$1 # run_algorithm_modm or run_algorithm_castar
algo=$2 # ABS or MODM or CASTAR or DFS

time_limit=$3
memory_limit=$4

head_seq=(2 5 10 15 20 30 40 50 60 80)
tail_seq=( $(seq 100 25 300) )
seq=("${head_seq[@]}" "${tail_seq[@]}")
# seq=(2 5)


for i in "${seq[@]}"; do

	cmd1="./$shell_script.sh $time_limit $memory_limit $algo small_obstacles_range3_3d_agents${i}_"
	cmd2="./$shell_script.sh $time_limit $memory_limit $algo pyramid_range3_3d_agents${i}_"
	cmd3="./$shell_script.sh $time_limit $memory_limit $algo offices_range1_agents${i}_"
	cmd4="./$shell_script.sh $time_limit $memory_limit $algo parallel_range4_agents${i}_"

	$cmd1 &
	$cmd2 & 
	$cmd3 & 
	$cmd4 &
	wait
done

python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p offices_range1_agents -l logs_$algo/ -t time > plots/$algo/office_time.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p parallel_range4_agents -l logs_$algo/ -t time > plots/$algo/parallel_time.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l logs_$algo/ -t time > plots/$algo/pyramid_time.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p small_obstacles_range3_3d_agents -l logs_$algo/ -t time > plots/$algo/smallobstacles_time.csv

python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p offices_range1_agents -l logs_$algo/ -t success > plots/$algo/office_success.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p parallel_range4_agents -l logs_$algo/ -t success > plots/$algo/parallel_success.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l logs_$algo/ -t success > plots/$algo/pyramid_success.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p small_obstacles_range3_3d_agents -l logs_$algo/ -t success > plots/$algo/smallobstacles_success.csv

python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p offices_range1_agents -l logs_$algo/ -t cost > plots/$algo/office_cost.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p parallel_range4_agents -l logs_$algo/ -t cost > plots/$algo/parallel_cost.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l logs_$algo/ -t cost > plots/$algo/pyramid_cost.csv
python3 generate_plot.py -a $algo -c CHECK_COLLISIONS -p small_obstacles_range3_3d_agents -l logs_$algo/ -t cost > plots/$algo/smallobstacles_cost.csv