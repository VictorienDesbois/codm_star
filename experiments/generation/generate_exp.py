import os
from igraph import *
import argparse
import random
import copy


output_folder = "/tmp/"


def generate_connected_configuration(comm, nb_agents, from_base=None):
    """
    Generate connected configuration of size nb_agents
    Argument from_base is the vertex of the first agent, which will serve as the basis.
    If it is None, then we pick it randomly
    """
    n = len(comm.vs)
    if from_base == None:
        prev_node = random.randint(0,n-1)
    else:
        prev_node = from_base
    start = [prev_node]
    cluster = set(map(lambda x: x.index, comm.vs[prev_node].neighbors()))
    for a in range(1,nb_agents):
        # resample if the chosen vertex is already occupied by another agent
        neighbors = list(cluster)
        if len(neighbors) <= a:
            #raise Exception("Cannot make connected configuration collision-free")
            return None
        while prev_node in start:
            prev_node = neighbors[random.randint(0,len(neighbors)-1)]
        cluster = cluster | set(map(lambda x: x.index, comm.vs[prev_node].neighbors()))
        start.append(prev_node)
    return start


def generate_sequentially_connected_configuration(comm, nb_agents, from_base=None):
    """
    Generate a random configuration where each agent i is connected to i-1
    """

    def generate_sequentially_connected_configuration_aux(conf, support, i):
        """
        @pre i>0
        @pre conf defined for 0...i-1
        returns True on success
        """
        if (i == len(conf)):
            return True
        cluster = list(set(map(lambda x: x.index, comm.vs[conf[i-1]].neighbors())) - support)
        random.shuffle(cluster)
        for v in cluster:
            support.add(v)
            conf[i] = v
            if(generate_sequentially_connected_configuration_aux(conf, support, i+1)):
                return True
            support.remove(v)
        return False
    

    n = len(comm.vs)
    if from_base is None:
        node = random.randint(0,n-1)
    else:
        node = from_base
    support = set([node])
    conf = [None] * nb_agents
    conf[0] = node
    if (generate_sequentially_connected_configuration_aux(conf, support, 1)):
        return conf
    else:
        return None


def generate_window_connected_configuration(comm, nb_agents, window):
    """
    Generate a random configuration where the following sets are connected:
    0,1,...,window-1
    window-1,window,...2(window-1)
    2(window-1),2window,...,3(window-1)
    etc.
    """
    def add_agent(conf, cluster, support, i):
        """
        @pre i>0
        @pre conf defined for 0...i-1
        returns True on success
        """
        if (i == len(conf)):
            return True
        candidates = set([])
        for neigh in map(lambda node: set(comm.vs[node].neighbors()), cluster):
            candidates = candidates | set(map(lambda n: n.index, neigh))
        candidates = list(candidates - support)

        random.shuffle(candidates)
        for v in candidates:
            support.add(v)
            conf[i] = v
            if ( (i % (window-1)) == 0 ):
                # then we want to be connected to the previous node only
                new_cluster = [v]
            else:
                new_cluster = copy.copy(cluster)
                new_cluster.append(v)
            if(add_agent(conf, new_cluster, support, i+1)):
                return True
            support.remove(v)
        return False
    n = len(comm.vs)
    node = random.randint(0,n-1)
    support = set([node]) # set of nodes already in the configuration
    cluster = set([node]) # set of nodes to which the next node is to be connected
    conf = [None] * nb_agents
    conf[0] = node
    if (add_agent(conf, cluster, support, 1)):
        return conf
    else:
        return None


def generate_connected_successor_rec(phy, comm, conf, partial_succ, support, i):
    if i == len(conf):
        return True
    move_neighbors = list(map(lambda x : x.index, phy.vs[conf[i]].neighbors()))
    move_neighbors.append(conf[i])
    conn_neighbors = set([])
    if i == 0:
        conn_neighbors =  set(move_neighbors)
    else:
        for j in range(0,i):
            conn_neighbors = conn_neighbors | set(map(lambda x: x.index, comm.vs[partial_succ[j]].neighbors()))
    # list of successors of conf[i] which are neighbors of partial_succ[0...i-1]
    # print("Move neighbors of node ", i, ": ", move_neighbors)
    # print("Comm neighbors of node ", i, ": ", conn_neighbors)
    candidate_neighbors = list((set(move_neighbors) & conn_neighbors) - support)
    # print("Viable neighbors of node ", i, ": ", len(candidate_neighbors))
    random.shuffle(candidate_neighbors)
    for v in candidate_neighbors:
        partial_succ[i] = v
        support.add(v)
        if (generate_connected_successor_rec(phy,comm,conf,partial_succ,support,i+1)):
            return True
        support.remove(v)
    return False


def generate_connected_successor(phy, comm, conf):
    """
    Generate a random connected successor of conf where agent i is connected to {0,1,...,i-1}
    """
    partial_succ = [None] * len(conf)
    if (generate_connected_successor_rec(phy, comm, conf, partial_succ, set([]), 0)):
        return partial_succ
    else:
        return None


def check_connected(comm, conf):
    u = conf[0]
    cluster = set(map(lambda x: x.index, comm.vs[u].neighbors()))
    for v in conf[1:]:        
        #print("Neighborshood of u=" + str(u) + ": ", neighbors)
        #print("Is v=", v, " in?")
        if not(v in cluster):
            return False
        u = v
        cluster = cluster | set(map(lambda x: x.index, comm.vs[u].neighbors()))
    return True


def generate(phys_filename, comm_filename, nb_agents, window_mode, window_size, filename):
    # phy = Graph.Read_GraphML(phys_filename)
    comm = Graph.Read_GraphML(comm_filename)
    if (window_mode == 2):
        print(f"Generating {window_size}-window-connected configurations")
        start = generate_window_connected_configuration(comm, nb_agents, window_size)
        goal = generate_window_connected_configuration(comm, nb_agents, window_size)
    if (window_mode == 1):
        print(f"Generating {window_size}-window-connected start, and arbitrary connected goal configurations")
        start = generate_window_connected_configuration(comm, nb_agents, window_size)
        goal = generate_connected_configuration(comm, nb_agents)
        # goal = generate_window_connected_configuration(comm, nb_agents, window_size)
    else:
        print("Generating connected configurations")
        start = None
        goal = None
        while (start is None):
            start = generate_connected_configuration(comm, nb_agents)
        while (goal is None):
            goal = generate_connected_configuration(comm, nb_agents)
    assert(check_connected(comm,start))
    assert(check_connected(comm,goal))
    # We have start[0] == goal[0]. This is the base agent's position
    with open(output_folder + filename,"w", encoding="utf-8") as f:
        print("Writing " + output_folder + filename)
        print("phys_graph " + os.path.basename(phys_filename), file=f)
        print("comm_graph " + os.path.basename(comm_filename), file=f)
        print("start ",end='',file=f)
        for v in start:
            print(str(v),"",end='',file=f)
        print("",file=f)
        print("goal ",end='',file=f)
        for v in goal:
            print(str(v),"",end='',file=f)
        print("",file=f)


def main():
    global output_folder
    parser = argparse.ArgumentParser(description="Exp generator. Given physical and communication graphs this script randomly generates two connected components and writes this in an .exp file. Agent 0 has the same start and goal vertex since it will serve as a fixed base.")
    parser.add_argument("-p", "--physical", type=str, dest="phys",
                        help="Physical graph",required=True)
    parser.add_argument("-c", "--communication", type=str, dest="comm",
                        help="Communication graph",required=True)
    parser.add_argument("-a", "--agents", dest="nb_agents", type=int,                        
                        help="Number of agents",required=True)
    parser.add_argument("-n", "--number_of_exps", dest="nb_exps", type=int,
                        help="Number of agents",required=True)
    parser.add_argument("-o",dest="out", type=str,
                        help="Output file name base",required=True)
    parser.add_argument("-d", dest="outdir", type=str, help="Output directory", required=True)
    parser.add_argument("-wm",dest="window_mode", type=int,
                        help="Whether the start and goal configurations are to be window connected. 0: none is window connected, 1: only start is window-connected, 2: both are window connected",required=False)
    parser.add_argument("-ws",dest="window_size", type=int, default=2,
                        help="window size for window-connected configurations.",required=False)
    args = parser.parse_args()

    nb_agents = args.nb_agents
    nb_exps = args.nb_exps
    window_mode = args.window_mode
    window_size = args.window_size
    output_folder = args.outdir+"/"

    random.seed()
    for i in range(nb_exps):
        filename = args.out + "_" + str(i) + ".exp"
        generate(args.phys, args.comm, nb_agents, window_mode, window_size, filename)

main()
