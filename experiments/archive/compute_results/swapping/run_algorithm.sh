#!/usr/bin/bash

time_limit=$1
memory_limit=$2
algo=$3
bench_prefix=$4

swapping_conflicts=$5

col=$6 # CHECK_COLLISIONS or IGNORE_COLLISIONS
nbst=$7

if [[ $nbst = '' ]]; then
    nbst=100
fi

ulimit -t ${time_limit} -v ${memory_limit} -m ${memory_limit}

# folders & files
GFOLDER="../../generation/graphs/"
EFOLDER="../../generation/data/"
logfile=logs_$algo/${bench_prefix}_${algo}_${col}
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.
rm -f $logfile
echo "$EFOLDER$bench_prefix"

# looping through all experiments
for SEL in `find $EFOLDER$bench_prefix*.exp`
do
    echo '
    ' >> $logfile.txt
    echo ">>>>> $SEL" >> $logfile.txt

    cmd="time ./../codm_solvers -e $SEL -g $GFOLDER -a $algo -v false -s $swapping_conflicts"

    if [ $algo == "CASTAR" ]
    then
        cmd="time ./../castar -a $algo -e $SEL -G $GFOLDER --collisions CHECK_COLLISIONS --nbst 100 --verbose false --subsolver CASTAR"
    elif [ $algo == "CASTAR-SWAPP" ]
    then
        cmd="time ./../castar_swapping -a $algo -e $SEL -G $GFOLDER --collisions CHECK_COLLISIONS --nbst 100 --verbose false --subsolver CASTAR"
    fi

    echo $cmd >> $logfile.txt
    echo $cmd
      $cmd >> $logfile.txt 2>&1
done
echo "Results written to $logfile.txt"

# echo rm -f -- $logfile.csv
if [ $algo == "CASTAR" ]
then
    python3 parse_log_castar.py $logfile.txt > $logfile.csv
else
    python3 parse_log_codm.py $logfile.txt > $logfile.csv
fi

