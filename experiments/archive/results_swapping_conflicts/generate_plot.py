import glob
import re
import os
import argparse
import collections


NB_AGENTS_MAX = 10000

def plot_success():
    # pat = re.compile(f"({prefix})([0-9]*)__(.*?)_.csv")
    pat = re.compile(f"({prefix})([0-9]*)__(.*?)_(.*).csv")
    files = []

    for absfile in glob.glob(logfolder + "/" + prefix + "*.csv"):
        file = os.path.basename(absfile)
        m = pat.match(file)
        assert(m)
        
        (exp, nb_agents, ealg, ecol) = m.groups()

        if ealg == alg:
            files.append((int(nb_agents), absfile)) 
        
    files.sort()
    
    if plottype == "success":
    
        print("Experiment ; nb_agents ; success_rate")

        for absfile in files:
            average_length = 0
            average_time = 0

            with open(absfile[1], mode='r') as csv_file:
                lines = list(csv_file.readlines())
                nb_lines = len(lines)

                if nb_lines <= 0:
                    nb_lines = 1

                success = 0
                for line in lines:
                    parts = line.split(" ; ")

                    id_parts = 1

                    if int(parts[id_parts]) == 1:
                        success += 1
            
            if absfile[0] <= NB_AGENTS_MAX:
                if nb_exp is None:
                    print(f"{exp} ; {absfile[0]} ; {success / float(nb_lines)}")
                else:
                    print(f"{exp} ; {absfile[0]} ; {success / float(nb_exp)}")

    if plottype == "time":
    
        print("id ; Experiment ; nb_agents ; time")

        time_dictionnary = {}

        for absfile in files:
            average_length = 0
            average_memory = 0

            with open(absfile[1], mode='r') as csv_file:
                lines = list(csv_file.readlines())
                nb_lines = len(lines)

                if nb_lines <= 0:
                    nb_lines = 1

                success = 0
                for line in lines:
                    parts = line.split(" ; ")

                    id_parts = 1

                    if int(parts[id_parts]) == 1:
                        success += 1
                        current_time = float(parts[3])
                        if current_time in time_dictionnary:
                            time_dictionnary[current_time].append([exp, absfile[0]])
                        else:    
                            time_dictionnary[current_time] = [[exp, absfile[0]]]

        ordered = collections.OrderedDict(sorted(time_dictionnary.items()))

        current_id = 0
        for k, v in ordered.items():
            for values in v:
                if values[1] <= NB_AGENTS_MAX:
                    print(f"{current_id} ; {values[0]} ; {values[1]} ; {k}")
                    current_id = current_id + 1

    if plottype == "cost":

        print("id ; Experiment ; nb_agents ; cost")

        cost_dictionnary = {}

        for absfile in files:
            average_length = 0
            average_memory = 0

            with open(absfile[1], mode='r') as csv_file:
                lines = list(csv_file.readlines())
                nb_lines = len(lines)

                if nb_lines <= 0:
                    nb_lines = 1

                success = 0
                for line in lines:
                    parts = line.split(" ; ")

                    id_parts = 1

                    if int(parts[id_parts]) == 1:
                        success += 1
                        current_time = float(parts[2])
                        if current_time in cost_dictionnary:
                            cost_dictionnary[current_time].append([exp, absfile[0]])
                        else:    
                            cost_dictionnary[current_time] = [[exp, absfile[0]]]

        ordered = collections.OrderedDict(sorted(cost_dictionnary.items()))

        current_id = 0
        for k, v in ordered.items():
            for values in v:
                if values[1] <= NB_AGENTS_MAX:
                    print(f"{current_id} ; {values[0]} ; {values[1]} ; {k}")
                    current_id = current_id + 1


def main():
    global alg, col, logfolder, prefix, caalone, plottype, nb_exp
    parser = argparse.ArgumentParser(description="Given prefix, alg and collision mode, read all csv files in given log directory, produce summary.")
    parser.add_argument("-a", "--algorithm", type=str, dest="alg",
                        help="Algorithm",required=True)
    parser.add_argument("-c", "--collision", type=str, dest="col",
                        help="collision mode",required=True)
    parser.add_argument("-p", "--prefix", type=str, dest="prefix",
                        help="Algorithm",required=True)
    parser.add_argument("-l",dest="logfolder", type=str,
                        help="log folder to read",required=False)
    parser.add_argument("-t",dest="plottype", type=str,
                        help="success or performance",required=True)
    parser.add_argument("-n",dest="total_exp_nb", type=int,
                        help="total nb of exps in each file",required=False)
    parser.add_argument("-q",dest="alone", type=bool,
                        help="whether to filter benchmarks that were not solved by ca* alone",required=False)
    args = parser.parse_args()

    alg = args.alg
    col = args.col
    if args.logfolder:
        logfolder = args.logfolder
    prefix = args.prefix    
    plottype = args.plottype
    caalone = args.alone
    nb_exp = args.total_exp_nb
    plot_success()

main()
