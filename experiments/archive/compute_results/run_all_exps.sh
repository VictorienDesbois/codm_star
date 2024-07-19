# defaults parameters:
# MEM_LIMIT=8000000, TIME_LIMIT=120
# ulimit -t ${TIME_LIMIT} -v ${MEM_LIMIT} -m ${MEM_LIMIT}

./run_experiments.sh 150 3000000 "false"
./run_experiments.sh 150 3000000 "true"

# cd ../results
# echo "Make the plots !"
# python3 make_plot_pictures.py

# cd ../results_swapping_conflicts
# python3 make_plot_pictures.py
