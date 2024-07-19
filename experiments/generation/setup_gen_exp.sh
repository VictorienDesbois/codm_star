set -e

# Number of instances per case
head_seq=(2 5 10 15 20 30 40 50 60 80)
tail_seq=( $(seq 100 25 300) )
full_seq=("${head_seq[@]}" "${tail_seq[@]}")

for i in "${full_seq[@]}"; do
	python3 generate_exp.py -d data -c graphs/obstacles.png_comm_uniform_grid_1_range_3.graphml -p graphs/obstacles.png_phys_uniform_grid_1_range_3.graphml -a $i -n 20 -o small_obstacles_range3_3d_agents${i}
	python3 generate_exp.py -d data -c graphs/pyramid.png_comm_uniform_grid_1_range_3_3d.graphml -p graphs/pyramid.png_phys_uniform_grid_1_range_3_3d.graphml -a $i -n 20 -o pyramid_range3_3d_agents${i}
	python3 generate_exp.py -d data -p graphs/offices.png_phys_uniform_grid_1_range_1.graphml -c graphs/offices.png_comm_uniform_grid_1_range_1.graphml -a $i -n 20 -o offices_range1_agents${i}
	python3 generate_exp.py -d data -p graphs/rooms.png_phys_uniform_grid_1_range_4.graphml -c graphs/rooms.png_comm_uniform_grid_1_range_4.graphml -a $i -n 20 -o parallel_range4_agents${i}
done

# parallel 2D
# offices 2D
#
# pyramid 3D
# obstacle_field 3D