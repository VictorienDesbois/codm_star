import sys
import re


def parse(filename):
    """
    parse the log file to print the time and cost of each runs
    """

    with open(filename,'r') as f:
        bench = ""

        for line in f:
            # each new line start by >>>>>
            m = re.search('>>>>> (.*)', line)
            if m:
                bench=m.group(1)
                time=0
                memory=0
                success=0
                longest_path=0

            m = re.search('Execution found!', line)
            if m:
                success = 1

            m = re.search('Longest path: ([0-9]+)', line)
            if m:
                longest_path=int(m.group(1))

            m = re.search('([0-9]+)maxresident', line)
            if m:
                memory = int(int(m.group(1)) / 1000) 

            m = re.search('(.*)user', line)
            if m:
                time = float(m.group(1))
                print(bench,";",success,";",longest_path,";",time,";",memory)


if len(sys.argv) < 2:
    raise Exception("Give log file name")

filename=sys.argv[1]
parse(filename)