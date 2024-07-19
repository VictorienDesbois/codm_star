import sys
import re


def parse(filename):

    with open(filename,'r') as f:

        bench = ""
        time = 0
        cost = 0

        for line in f:
            m = re.search('>>>>> (.*)', line)
            if m:
                bench=m.group(1)
                time=0
                cost=0
                castar_alone = 0
                    
            m = re.search('Longest path: ([0-9]+)', line)
            if m:
                cost = int(m.group(1))

            m = re.search('(.*)user', line)
            if m:
                time = float(m.group(1))

                # pass # if printed cost is 0, this means the benchmark failed
                if cost > 0:
                    print(bench,"; 1 ;",cost,";",time)
                else:
                    print(bench,"; 0 ;",cost,";",time)            

if len(sys.argv) < 2:
    raise Exception("Give log file name")

filename=sys.argv[1]
parse(filename)
