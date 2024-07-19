#!/usr/bin/bash

time_limit=$1
memory_limit=$2
algo=$3
bench_prefix=$4

col=$5 # CHECK_COLLISIONS or IGNORE_COLLISIONS
nbst=$6

if [[ $nbst = '' ]]; then
    nbst=100
fi

ulimit -t ${time_limit} -v ${memory_limit} -m ${memory_limit}

# folders & files
GFOLDER="../generation/graphs/"
EFOLDER="../generation/data/"
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

    cmd=""

    if [ $algo == "CODMOPTI" ]
    then
        cmd="time ./codm -e $SEL -g $GFOLDER -o true"
    elif [ $algo == "CODM" ]
    then
        cmd="time ./codm -e $SEL -g $GFOLDER -o false"
    elif [ $algo == "CASTAR" ]
    then
        cmd="time ./castar -a $algo -e $SEL -G $GFOLDER --collisions CHECK_COLLISIONS --nbst 100 --verbose false --subsolver CASTAR"
    fi

    echo $cmd >> $logfile.txt
    echo $cmd
      $cmd >> $logfile.txt 2>&1
done
echo "Results written to $logfile.txt"

# echo rm -f -- $logfile.csv
if [ $algo == "CASTAR" ]
then
    python3 parse_log.py $logfile.txt > $logfile.csv
else
    python3 parse_log_abs.py $logfile.txt > $logfile.csv
fi

