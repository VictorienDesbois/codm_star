/* Copyright (c) 2022 Ocan Sankur
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 */
#pragma once

#include <Execution.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <DFS.hpp>
#include <ShortestPathCalculator.hpp>
#include <Options.hpp>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <list>
namespace decoupled{

/**
 * @brief CA* algorithm. The compute method attempts to find the longest connected and collision-free execution towards the goal configuration,
 * by trying \a number_of_runs different random orderings of the agents. 
 * This abstract class provides \a computePrefix function which implements the CA* algorithm to compute an execution towards goal.
 * 
 * @tparam GraphMove 
 * @tparam GraphComm 
 */

template <class GraphMove, class GraphComm>
class CAStar : public Solver<GraphMove, GraphComm>
{
protected:
    coupled::DFS<GraphMove, GraphComm> dfs_solver_;
    std::vector<Configuration> config_stack_;
    //DijkstraSPCalculator<GraphMove,GraphComm> dijkstra_;
    AStarSPCalculator<GraphMove,GraphComm> dijkstra_;
    Heuristics<GraphMove, GraphComm> & heuristics_;
    bool verbose_ = false;
public:
    CAStar(const Instance<GraphMove, GraphComm> &instance, const Objective &objective, Heuristics<GraphMove, GraphComm>& heuristics, bool verbose)
        : Solver<GraphMove, GraphComm>(instance, objective), dfs_solver_(instance, objective, heuristics, false), dijkstra_(instance), heuristics_(heuristics), verbose_(verbose) {}
    ~CAStar() {}

private:

        /**
         * @brief Serch node for the CA* algorithm. Each node is given with its successor, whether from the successor
         * there is an actual path to goal (\a has_path), the length of the feasible suffix from the current node (\a suffix_length),
         * the h value (feasible suffix + heuristic value), and whether the current node has no successor in the next frame (\a dead_end)
         * 
         */
        class CANode {
            public:
            CANode(){
                node = std::numeric_limits<Node>::max(); // undefined
            }
            CANode(Node node, Node successor, bool has_path, size_t suffix_length, size_t h, bool dead_end = false) : node(node), successor(successor), has_path(has_path), suffix_length(suffix_length), h(h), dead_end(dead_end){
            }
            Node node;
            Node successor;
            bool has_path;
            size_t suffix_length;
            size_t h;
            bool dead_end;
            bool operator<(const CANode & b) const
            {
                return this->has_path > b.has_path
                        || this->has_path && b.has_path && (this->suffix_length < b.suffix_length)
                        || !this->has_path && !b.has_path && (b.suffix_length < this->suffix_length)
                        || this->has_path == b.has_path && (this->suffix_length == b.suffix_length) && this->h < b.h;
            }
            void print(std::ostream & os){
                os << "* canode " << node << " -> " 
                                << successor << ", has_path: " << has_path << ", suffix_length: " << suffix_length
                                << ", h: " << h << "\n";        
            }
            inline bool defined() const {
                return node < std::numeric_limits<Node>::max();
            }
        };


    /**
     * @brief Check if two positions of two agents generate a swapping
     * conflict inside the path collection.
     */
    bool is_position_a_swapping_conflict(
      Node previous_p,
      Node current_p,
      std::vector<Path> &paths,
      size_t time_step
    ) {

      size_t current_time_step;
      size_t previous_time_step;

      for (Path path: paths) {

        if (path.size() > 1) {
          if (path.size() <= time_step) {
            previous_time_step = path.size() - 2;
            current_time_step = path.size() - 1;
          } else {
            previous_time_step = time_step - 1;
            current_time_step = time_step;
          }

          if (path.at(previous_time_step) == current_p && 
              path.at(current_time_step) == previous_p) {
            return true;
          }
        }
      }

      return false;
    }

    /**
     * @brief Find the longest such that at each step t, the node is in \a neighborhoods[t], while either actually reaching goal,
     * or minimizing the h-value to goal at the last vertex.
     *
     * Each set neighborhoods[t] is called a frame. We first compute the set of nodes in the last frame which has a path to goal
     * which stays in this frame. These nodes have an actual path to goal; and the execution can be extended for agt to reach goal
     * by making all other agents idle.
     * Then we compute the best path from each node of each frame backwards. At each frame t, and node n, we assign to n the successor
     * from frame t+1 which has, if possible, an actual path to goal and in this case the one admitting the shortest one,
     * and otherwise a successor node with the longest path with the least h value. Some nodes of frame t might have no successors in
     * frame t+1. These are marked as deadends, and assigned a heuristics h value.
     * At the end of the computation, the start node at frame 0 has been assigned the best path. We follow this path, and possibly 
     * complete it with a suffix path within the last frame.
     * 
     * @pre Agent \a agt staying inside \a neighborhoods must ensure connectivity with agents 0 .. \a agt -1, and collision-freedom.
     * @param i 
     * @param neighborhoods 
     * @return Path 
     */
    Path getConnectedPathTowardsGoal(
        Configuration startConf, 
        Agent agt, 
        std::vector<std::unordered_set<Node>> & neighborhoods,
        std::vector<Path> & paths){

        bool local_verbose = false;
        Node start = startConf.at(agt);
        Node goal = this->instance_.goal().at(agt);
        Path p;
        
        // std::cout << "\n* getConnectedPathTowardsGoal(Agt: " << agt << " Start: " << start << ", goal: " << goal << ") with neighborhoods size: " <<
        //     neighborhoods.size() << "\n";
        // std::cout << "neighborhoods[0] = ";
        // for (auto node : neighborhoods[0]){
        //     std::cout << node << " ";
        // }
        // std::cout << "\n";  

        // if (neighborhoods.size()>1){
        //     std::cout << "neighborhoods[1] = ";
        //     for (auto node : neighborhoods[1]){
        //         std::cout << node << " ";
        //     }
        //     std::cout << "\n";  
        // }

        // std::cout << "neighborhoods[" << neighborhoods.size()-1 << "] = ";
        // for (auto node : neighborhoods[neighborhoods.size()-1]){
        //     std::cout << node << " ";
        // }
        // std::cout << "\n";  
        assert(neighborhoods[0].contains(start));

        if (local_verbose){
            std::cout << "Agt: " << agt << " Start: " << start << ", goal: " << goal << "\n";
        }
        // We start by computing the suffix. Let suffix_h be the distance from each vertex of the last frame to goal,
        // while staying inside the said frame. Let suffix_next be the next node to go to follow this suffix.
        // The path that follows suffix_next corresponds to previous agents idling at their final positions while agt continuing towards their goal.

        // auto suffix_next = std::make_unique<Node[]>(this->instance_.graph().movement().node_count());
        // auto suffix_h = std::make_unique<size_t[]>(this->instance_.graph().movement().node_count());

        std::vector<Node> suffix_next;
        std::vector<size_t> suffix_h;
        for(int i = 0; i < this->instance_.graph().movement().node_count(); i++){
            suffix_next.push_back(std::numeric_limits<Node>::max());
            suffix_h.push_back(std::numeric_limits<size_t>::max());
        }
        //     suffix_next[i] = std::numeric_limits<Node>::max();
        //     suffix_h[i] = std::numeric_limits<size_t>::max();
        // }
        // std::unordered_map<Node,Node> suffix_next; // for a vertex of the last frame, the vertex to go to for agt
        // std::unordered_map<Node,size_t> suffix_h; // for a vertex of the last frame, the distance to go for agt from given node to goal
        suffix_next.at(goal)= goal;
        suffix_h.at(goal) = 0;
        std::stack<Node> wait;
        // auto has_path_to_goal_v = std::make_unique<bool[]>(this->instance_.graph().movement().node_count());
        std::vector<bool> has_path_to_goal_v;
        for(int i = 0; i < this->instance_.graph().movement().node_count(); i++){
            has_path_to_goal_v.push_back(false);
        }
        std::vector<Node> has_path_to_goal_v_support; // vector of nodes node such that has_path_to_goal_v[node] = true

        // std::unordered_set<Node> has_path_to_goal;
        // We do this step only if goal is in the last frame
        if (neighborhoods.at(neighborhoods.size()-1).contains(goal))
            wait.push(goal);

        while(!wait.empty()){
            Node current = wait.top();
            wait.pop();
            if (has_path_to_goal_v.at(current))
                continue;
            // if (has_path_to_goal.contains(current))
            //     continue;
            // // has_path_to_goal.insert(current);
            has_path_to_goal_v.at(current) = true;
            has_path_to_goal_v_support.push_back(current);
            for (Node sucnode : this->instance_.graph().movement().get_neighbors(current)){
                // for all movement neighbor node sucnode that are in the last frame
                if (neighborhoods[neighborhoods.size()-1].contains(sucnode) && !has_path_to_goal_v.at(sucnode)){
                // if (neighborhoods[neighborhoods.size()-1].contains(sucnode) && !has_path_to_goal.contains(sucnode)){
                    suffix_next.at(sucnode) = current;
                    suffix_h.at(sucnode) = suffix_h.at(current) + 1;
                    if (local_verbose){
                        std::cout << "suffix_next[" << sucnode << "] = " << current << "\n";
                        std::cout << "suffix_h[" << sucnode << "] = " << suffix_h[sucnode] <<"\n";
                    }
                    wait.push(sucnode);
                }
            }
        }

        // std::vector<std::unordered_map<Node,CANode> > prefix_h;
        // std::vector<std::unique_ptr<CANode[]>> prefix_h;
        std::vector<std::vector<CANode>> prefix_h;
        for(int i=0; i < neighborhoods.size(); i++){
            // prefix_h.push_back(std::make_unique<CANode[]>(this->instance_.graph().movement().node_count()));
            prefix_h.push_back(std::vector<CANode>(this->instance_.graph().movement().node_count()));
        }
        // for(int i=0; i < neighborhoods.size(); i++){
        //     prefix_h.push_back(std::unordered_map<Node,CANode>());
        // }

        for (Node node : has_path_to_goal_v_support){
            prefix_h.at(neighborhoods.size()-1).at(node) = CANode(node, node, true, suffix_h.at(node), suffix_h.at(node));
        }
        // for(Node node : has_path_to_goal){
        //     prefix_h[neighborhoods.size()-1][node] = CANode(node, node, true, suffix_h[node], suffix_h[node]);
        // }
        for (Node node : neighborhoods[neighborhoods.size()-1]){
            // if (!has_path_to_goal.contains(node)){
            if (!has_path_to_goal_v[node]){
                prefix_h[neighborhoods.size()-1][node] = CANode(node, node, false, 0, this->heuristics_.getHeuristic(node,goal));
            }
        }
        if (local_verbose){
            std::cout << "Last frame (index " << neighborhoods.size()-1 << ") is\n";
            for (Node node : neighborhoods[neighborhoods.size()-1]){
                CANode canode = prefix_h[neighborhoods.size()-1][node];
                canode.print(std::cout);
            }
        }
        for (int i = neighborhoods.size()-1; i> 0; i--){
            for (Node node : neighborhoods[i-1]){
                for (Node sucnode : this->instance_.graph().movement().get_neighbors(node)){
                    if (!neighborhoods[i].contains(sucnode))
                        continue;

                    // check swapping conflicts
                    if (is_position_a_swapping_conflict(node, sucnode, paths, i))
                        continue;

                    // assert(prefix_h[i].count(sucnode) != 0);
                    CANode sucCANode = prefix_h[i][sucnode];
                    // std::cout << "Successor is: "; sucCANode.print(std::cout);

                    CANode candidate(node, sucnode, sucCANode.has_path, sucCANode.suffix_length+1, sucCANode.h+1);
                    // if (prefix_h[i-1].count(node) == 0 
                    if (!prefix_h[i-1][node].defined()
                        || candidate < prefix_h[i-1][node])
                    {
                        prefix_h[i-1][node] = candidate;
                        if (local_verbose){
                            //std::cout << "prefix_next(" << i-1 << ")[node " << node << "] = node " << sucnode << "\n";
                            std::cout << "prefix_h[" << i-1 << "] "; candidate.print(std::cout);
                        }
                    }
                }
                // if node has no successors in neighborhoods[i], then it is a dead end
                // if (prefix_h[i-1].count(node) == 0){
                if (!prefix_h[i-1][node].defined()){
                    CANode candidate(node, node, false, 0, this->heuristics_.getHeuristic(node,goal), true);
                    prefix_h[i-1][node] = candidate;
                }
            }
        }

        // assert(prefix_h[0].count(start) > 0);
        assert(prefix_h[0][start].defined());
        CANode canode = prefix_h[0][start];
        if (local_verbose)
            std::cout << "Prefix (" << start << " to " << goal << "): " << canode.node << " ";
        p.push_back(canode.node);
        for(int i = 1; !canode.dead_end && i < neighborhoods.size();i++){
            // assert(prefix_h[i].count(canode.successor)>0);
            assert(prefix_h[i][canode.successor].defined());
            canode = prefix_h[i][canode.successor];
            if (local_verbose){
                std::cout << canode.node << "@" << i << " ";
                std::cout.flush();
            }
            p.push_back(canode.node);
        }

        // check
        // for(int i = 0; i < p.size(); i++){
        //     assert(neighborhoods.at(i).contains(p[i]));
        // }

        // if (!canode.dead_end){
        //     p.push_back(canode.successor);
        //     std::cout << canode.successor << "$" << neighborhoods.size() << " ";
        //     std::cout.flush();
        // }

        // check 
        // Node node = p[0];
        // for(int i = 1; i < p.size(); i++){
        //     if (!this->instance_.graph().movement().get_neighbors(node).contains(p[i])){
        //         std::cout << "Prefix Step " << i << " is not a neighbor of previous step\n";
        //         exit(-1);
        //     }
        //     node = p[i];
        // }

        if (canode.has_path){
            if ( local_verbose){
                std::cout << " :: "; std::cout.flush();
            }
            Node node = canode.successor;
            while(node != goal){
                node = suffix_next[node];
                p.push_back(node);
                // assert(neighborhoods[neighborhoods.size()-1].contains(node));
                if ( local_verbose)
                    std::cout << node << " ";
            }
        }
        if ( local_verbose){
            std::cout << "\n"; std::cout.flush();
        }
        // check
        // node = p[0];
        // for(int i = 1; i < p.size(); i++){
        //     if (!this->instance_.graph().movement().get_neighbors(node).contains(p[i])){
        //         std::cout << "Whole Step " << i << " is not a neighbor of previous step\n";
        //         std::cout << "node " << p[i] << " not neighbor of node" << node << "\n";
        //         exit(-1);
        //     }
        //     node = p[i];
        // }
        return p;
    }


    protected:
    /**
     * @brief This is a best-effort CA* procedure that attempts to find the longest path towards goal configuration:
     * Pick random order, fill in config_stack_ by a path of Agent 1, then Agent 2 connected to 1, then 3 connected to 1,2 etc.
     * 
     */
    std::vector<Configuration> computePrefix(const Configuration & start, const Configuration & goal, int max_path_size = -1)
    {
        // std::cout << "* computePrefix(" << start;
        // std::cout << ")\n";

        std::vector<Agent> shuffled;
        for(Agent agt = 0; agt < this->instance_.nb_agents(); agt++){
            shuffled.push_back(agt);
        }
        std::random_shuffle(shuffled.begin(), shuffled.end());

        std::unordered_set<Node> cluster = this->instance_.graph().communication().get_neighbors(start.at(shuffled.at(0)));
        for(Agent i = 1; i < this->instance_.nb_agents(); i++){
            // Make sure that shuffle[agt] is connected to shuffle[0...agt-1], swap with someone if needed
            Agent j = i;
            while (!cluster.contains(start.at(shuffled[j])) && j < this->instance_.nb_agents()){
                j++;
            }
            assert(cluster.contains(start.at(shuffled[j])));
            Agent tmp = shuffled[i];
            shuffled[i] = shuffled[j];
            shuffled[j] = tmp;
            for(auto node : this->instance_.graph().communication().get_neighbors(start.at(shuffled[i]))){
                cluster.insert(node);
            }
        }

        // Safety check for debugging
        cluster = this->instance_.graph().communication().get_neighbors(start.at(shuffled[0]));
        for(Agent i = 1; i < this->instance_.nb_agents(); i++){
            assert(cluster.contains(start.at(shuffled[i])));
            for(auto node : this->instance_.graph().communication().get_neighbors(start.at(shuffled[i]))){
                cluster.insert(node);
            }
        }

        // Add first path
        std::vector<Path> paths;
        paths.push_back(this->dijkstra_.getShortestPath(start.at(shuffled[0]), goal.at(shuffled[0])));
        // std::cout << paths.back();
        // std::cout << "\nStart: " << start[shuffled[0]] << ". Goal: " << goal[shuffled[0]] << "\n";
        // assert(paths.back().front() == start[shuffled[0]]);
        // assert(paths.back().back() == goal[shuffled[0]]);
        if (max_path_size >0){
            if (paths.back().size() > max_path_size){
                paths.back().resize(max_path_size);
            }
        }
        int path_size = paths.back().size();

        // std::cout << ANSI_GREEN << "Got path for Agent " << shuffled[0] << " (number " << 0 << ") of size : " << paths.back().size() << ". From "
        //     << this->instance_.start()[shuffled[0]] << " to " << this->instance_.goal()[shuffled[0]] << "\n<" << ANSI_RESET;
        // for(auto node : paths.back()){
        //     std::cout << node << ",";
        // }
        // std::cout <<">\n";
        
        // Initialize neighborhoods: neighborhoods[t] is the set of unoccupied vertices that communicate with some vertex occupied by
        // some agent at time t. Any subsequent agent must be and can be somewhere in neighboorhoods[t].
        std::vector<std::unordered_set<Node>> neighborhoods(paths.back().size());
        // for(int t = 0; t < paths.back().size(); t++){
        //     neighborhoods.push_back(std::unordered_set<Node>());
        // }
        for(int t = 0; t < paths.back().size(); t++){
            // Add the comm. neighborhood of the new path
            for(auto node : this->instance_.graph().communication().get_neighbors(paths.back().at(t))){
                neighborhoods.at(t).insert(node);
            }
            if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS ){
                neighborhoods.at(t).erase(paths.back().at(t));
            }
            // std::cout << "Neighborhood[" << t<< "] = ";
            // for(auto node : neighborhoods[t]){
            //     std::cout << node << " ";
            // }
            // std::cout << "\n";
        }
        // Add other paths one by one
        // std::cout << "\nconnected_path_towards..."; std::cout.flush();
        for(Agent i = 1; i < this->instance_.nb_agents(); i++){
            // std::cout << "\nconnected_path_towards..."; std::cout.flush();
            paths.push_back(this->getConnectedPathTowardsGoal(start, shuffled.at(i), neighborhoods, paths));
            if (max_path_size > 0){
                if (paths.back().size() > max_path_size){
                    paths.back().resize(max_path_size);
                }
            }
            // std::cout << " done\n"; std::cout.flush();
            // std::cout << "\t Got path for agent " << shuffled[i] << ": ";
            // for(Node n : paths.back()){
            //     std::cout << n << " ";
            // }
            // std::cout << "\n";
            

            // std::cout << "updating..."; std::cout.flush();
            if(paths.back().size() > path_size){
                // If the added path is longer than the previous ones, extend the neighborhoods by assuming all others idle at their
                // last positions. Also extend the paths that are shorter than the new size.

                path_size = paths.back().size();
                for (int t = neighborhoods.size(); t < path_size; t++){
                    neighborhoods.push_back(neighborhoods.back());
                }
                for(int i = 0; i < paths.size(); i++){
                    if (paths.at(i).size() < path_size){
                        for(int t = paths.at(i).size(); t < path_size; t++){
                            paths.at(i).push_back(paths[i].back());
                        }

                    }
                }
            } else {
                // If the added path is shorter, than shorten all previous paths and neigborhoods
                path_size = paths.back().size();
                neighborhoods.resize(path_size);
                for(int i = 0; i <paths.size();i++){
                    paths.at(i).resize(path_size);
                }
            }
            // for(Node n : paths.back()){
            //     std::cout << n << " ";
            // }
            // std::cout << "\n";
            // std::cout << "path size is " << path_size << "\n";

            // std::cout << ANSI_GREEN << "Got path for Agent " << shuffled[i] << " (number " << i << ") of size : " << paths.back().size() << ANSI_RESET
            //     << " from " << this->instance_.start()[shuffled[i]] << " to " << this->instance_.goal()[shuffled[i]] << "\n<";
            // for(auto node : paths.back()){
            //     std::cout << node << ",";
            // }
            // std::cout <<">\n";

            for(int t = 0; t < neighborhoods.size(); t++){
                // Add the comm. neighborhood of the new path
                for(auto node : this->instance_.graph().communication().get_neighbors(paths.back().getAtTimeOrLast(t))){
                    neighborhoods.at(t).insert(node);
                }
                if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS ){
                    for (Agent j = 0; j <= i; j++){
                        neighborhoods.at(t).erase(paths.at(j).getAtTimeOrLast(t));
                    }
                }
            }
        }
        // std::cout << "done\n"; std::cout.flush();

        assert(paths.size() == this->instance_.nb_agents());
        // for(Agent i = 0; i < this->instance_.nb_agents(); i++){
        //     assert(paths[i].size()>0);
        //     std::cout << "Path of " << shuffled[i] << ": ";
        //     for(Node n : paths[i]){
        //         std::cout << n << " ";
        //     }
        //     std::cout << "\n";
        // }

        // Convert to vector of Configurations
        std::vector<Configuration> configSeq;
        for(int t = 0; t < path_size; t++){
            // std::cout << "paths[" << t << "].size() == " << paths[t].size() << "\n";
            Configuration c(this->instance_.nb_agents());
            for(Agent i = 0; i < this->instance_.nb_agents(); i++){
                c.at(shuffled.at(i)) = paths.at(i).getAtTimeOrLast(t);
            }
            if (this->instance_.graph().communication().isConfigurationConnected(c)){
                configSeq.push_back(c);
            } else {
                break;
            }
            if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS && c.hasCollisions()){
                std::cout << " [error] Collision at step " << t << ": " << c;
                std::cout <<"\n";
                assert(!c.hasCollisions());
            }
        }
        assert(configSeq.front() == start);
        return configSeq;
    }
};


/**
 * @brief Instance of the CA* algorithm where we attempt \a number_of_runs runs towards goal, and if goal was not reached, then we finish with the DFS solver.
 * 
 * @tparam GraphMove 
 * @tparam GraphComm 
 */
template <class GraphMove, class GraphComm>
class CAStarDFSFinish : public CAStar<GraphMove, GraphComm>
{
private:
    int number_of_runs_ = 100;
    int number_of_extrials_ = 100;

public:
    CAStarDFSFinish(const Instance<GraphMove, GraphComm> &instance, const Objective &objective, Heuristics<GraphMove, GraphComm>& heuristics, int number_of_runs, int number_of_extrials, bool verbose)
        : CAStar<GraphMove, GraphComm>(instance, objective, heuristics, verbose){}
    ~CAStarDFSFinish() {}

    const Execution compute() override
    {
        std::vector<Configuration> bestPrefix;
        bool goal_reached = false;
        for(int i = 0; i < this->number_of_runs_ && !goal_reached; i++){
            std::cout << ANSI_PURPLE << "trial number " << i << "\t " << ANSI_RESET;
            std::cout.flush();
            this->config_stack_.clear();
            // std::cout << ANSI_PURPLE << "\n\ncomputePrefix:\n" << ANSI_RESET;
            // for (int j = 0; j < 1; j++){
            //     const Configuration start = this->instance_.start();
            //     std::vector<Path> paths = computePrefix(start);
            // }
            Configuration start = this->instance_.start();
            std::vector<Configuration> prefix = this->computePrefix(start, this->instance_.goal());
            // std::cout << "(got prefix of size " << prefix.size() << ")\n";
            // std::cout << " ending in " << prefix.back() << "\n";
            // Try to extend it further
            if (prefix.back() != this->instance_.goal()){
                for (int j = 0; j < this->number_of_extrials_; j++){
                    std::vector<Configuration> segment = this->computePrefix(prefix.back(), this->instance_.goal());
                    if (segment.size()>1){
                        // std::cout << "(extending prefix by segment of size " << segment.size() << ")\n";
                        for(auto c = segment.begin()+1; c != segment.end(); c++){
                            prefix.push_back(*c);
                        }
                    }
                }
            }
            if (prefix.size() > bestPrefix.size()){
                bestPrefix = prefix;
                std::cout << ANSI_CYAN << "Got new prefix of length " << prefix.size() << "\n" << ANSI_RESET;
                std::cout << "\t" << bestPrefix.back();
                std::cout << "\n";
                if (bestPrefix.back() == this->instance_.goal()){
                    goal_reached = true;
                    break;
                }
            }
        }
        this->config_stack_ = bestPrefix;

        std::vector<std::shared_ptr<Configuration> > suffix;
        if (this->config_stack_.back() != this->instance_.goal()){
            if(this->verbose_){
                std::cout << ANSI_CYAN << "Keeping prefix of length " << bestPrefix.size() << "\n" << ANSI_RESET;
                std::cout << ANSI_BLUE << "Now running DFS from " << bestPrefix.back();
                std::cout << "\t towards " << this->instance_.goal();
                std::cout << "\n" << ANSI_RESET;
            }
            suffix = this->dfs_solver_.computeBoundedPathTowards(bestPrefix.back(), this->instance_.goal(), 1000000, false);
            std::cout << ANSI_CYAN << "Got suffix of length " << suffix.size() << "\n" << ANSI_RESET;
        } else {
            std::cout << ANSI_GREEN << "CA* solved the instance alone.\n" << ANSI_RESET;
        }

        if (suffix.size() > 0){
            for (auto it = suffix.begin()+1; it != suffix.end(); it++){
                this->config_stack_.push_back(**it);
            }
        }

        // Check
        assert(this->config_stack_.front() == this->instance_.start());
        assert(this->config_stack_.back() == this->instance_.goal());
        for(auto c : this->config_stack_){
            this->instance_.graph().communication().isConfigurationConnected(c);
        }

        // Create the execution
        for (int i = 0; i < this->instance_.nb_agents(); i++)
        {
            std::shared_ptr<Path> path = std::make_shared<Path>();
            for (auto c : this->config_stack_)
            {
                path->push_back(c[i]);
            }
            // if (suffix.size() > 0){
            //     for (auto it = suffix.begin()+1; it != suffix.end(); it++)
            //     {
            //         path->push_back((*it)->at(i));
            //     }
            // }
            assert(path->isValid(this->instance_.graph().movement()));
            this->execution_.push_back(path);
        }
        return this->execution_;
    }
};


/**
 * @brief Instance of the CA* algorithm where we attempt an unbounded number of runs. At each run,
 * if goal was not reached, we shuffle the order and try to extend the execution \a number_of_extrials times.
 * If in the last \a shake_threshold_ runs, we failed to improve the execution (that is, the found execution was
 * not longer than the previous best), then we attempt a "shaked run". A shaked run is defined as follows:
 * Pick a random connected configuration and run either CA* or DFS towards it for \a shake_length_ steps, depending
 * ont the shake_mode_;
 * retry \a shake_trials_ times if no steps can be made. Then run CA* from the current configuration towards goal.
 * 
 * @tparam GraphMove 
 * @tparam GraphComm 
 */

template <class GraphMove, class GraphComm>
class CAStarShake : public CAStar<GraphMove, GraphComm>
{
public: 
    enum class ShakeMode : int
    {
        CASTAR,
        DFS
    };
private:
    int shake_threshold_ = 5; // Nb of trials after which we will shake
    int shake_length_ = 10;   // Size of the shaking prefix before we try again
    int shake_trials_ = 25;   // Number of times we try to shake with CA*
    int number_of_extrials_ = 100;
    bool random_shaking_ = true;
    SubsolverEnum subsolver_;
public: 

    CAStarShake(const Instance<GraphMove, GraphComm> &instance, const Objective &objective, Heuristics<GraphMove, GraphComm>& heuristics, int number_of_extrials, SubsolverEnum subsolver, bool random_shaking, bool verbose)
        : CAStar<GraphMove, GraphComm>(instance, objective, heuristics, verbose), number_of_extrials_(number_of_extrials), subsolver_(subsolver), random_shaking_(random_shaking){}
    ~CAStarShake() {}


    Configuration getRandomConfiguration()
    {
        // Pick a node of Gc for the 1st agent
        size_t nb_nodes = this->instance().graph().communication().node_count();
        Node first = (uint64_t)rand() % (nb_nodes - 1);
        Configuration config;
        for (size_t agt = 0; agt < this->instance().nb_agents(); agt++)
            config.push_back(first);
        /*
        std::unordered_set<Node> neighbors;
        // Pick in the Gc neighbors the position for the 2nd, etc
        for (size_t agt = 1; agt < this->instance().nb_agents(); agt++)
        {
          // std::cout << "Size of config: " << config.size() << ", agt = " << agt << "\n";
          auto new_neighbors = this->instance().graph().communication().get_neighbors(
            config.back());
            //   config.at(agt - 1));
          neighbors.insert(new_neighbors.begin(), new_neighbors.end());

          auto neighbors_vector =
              std::vector<Node>(neighbors.begin(), neighbors.end());
          auto size = neighbors_vector.size();
          // std::cout << "size for agt " << agt << ":" << size;
          auto nextagt = rand() % size;
          config.push_back(neighbors_vector.at(nextagt));
          assert(neighbors_vector.at(nextagt) <
                 this->instance().graph().movement().node_count());
        }
        std::random_shuffle(config.begin(), config.end());
        */
        return config;
    }


    const Execution compute() override
    {
        std::vector<Configuration> prefix;
        std::vector<Configuration> bestSuffix;
        const Configuration & start = this->instance_.start();
        bool goal_reached = false;
        int i = 0;
        while(!goal_reached){
            i++;
            std::cout << ANSI_RED << "Run number " << i << "\n " << ANSI_RESET;
            std::cout.flush();
            this->config_stack_.clear();

            if (this->random_shaking_ && i >= this->shake_threshold_){
                prefix.clear();
                int st = 0;
                if(this->subsolver_ == SubsolverEnum::CASTAR){
                    // Shaking step: try random directions number_of_extrials_ times
                    for (st = 0; prefix.size() < this->shake_length_ && st < this->shake_trials_; st++){
                        // std::cout << ANSI_BLUE << "\nShaking with CA*: " << st << "\n"  << ANSI_RESET;
                        Configuration intermediate_goal = this->getRandomConfiguration();
                        auto prefix_candidate = this->computePrefix(start, intermediate_goal, this->shake_length_);
                        if (prefix_candidate.size() > prefix.size()){
                            prefix = prefix_candidate;
                        }
                    }
                    std::cout << ANSI_PURPLE << "\nShooked with CA*: " << st << " times\n"  << ANSI_RESET;
                } else if (this->subsolver_ == SubsolverEnum::DFS){
                    std::cout << ANSI_BLUE << "\nShaking with DFS\n" << ANSI_RESET;
                    Configuration intermediate_goal = this->getRandomConfiguration();
                    std::vector<std::shared_ptr<Configuration> > dfs_prefix =
                        this->dfs_solver_.computeBoundedPathTowards(start, intermediate_goal, this->shake_length_/2);
                    prefix.clear();
                    for(auto c : dfs_prefix){
                        prefix.push_back(*c);
                    }
                } else {
                    throw std::runtime_error("Subsolver not supported for CASTAR");
                }
                std::cout << ANSI_PURPLE << "Got prefix of size " << prefix.size()  << ". Now running CA* from new source\n" << ANSI_RESET;
            } else {
                prefix = {this->instance_.start()};
            }
            std::vector<Configuration> suffix = this->computePrefix(prefix.back(), this->instance_.goal());

            // Try to extend it further
            if (suffix.back() != this->instance_.goal()){
                int extension_progress_ago = 0;
                if (this->verbose_)
                    std::cout << "Attempting to extend prefix of size " << prefix.size() << "\n";
                for (int j = 0; j < this->number_of_extrials_; j++){
                    extension_progress_ago++;
                    if (suffix.back() == this->instance_.goal()){
                        bestSuffix = suffix;
                        std::cout << ANSI_GREEN << "CA*-Shake solved the instance.\n" << ANSI_RESET;
                        goal_reached = true;
                        break;
                    }
                    std::vector<Configuration> segment = this->computePrefix(suffix.back(), this->instance_.goal());
                    if (segment.size()>1){
                        if (this->verbose_)
                            std::cout << "\t(extending prefix of size " << prefix.size()<< " by segment of size " << segment.size() << ")\n";
                        for(auto c = segment.begin()+1; c != segment.end(); c++){
                            suffix.push_back(*c);
                        }
                        extension_progress_ago = 0;
                    } else if (this->random_shaking_ && extension_progress_ago > this->number_of_extrials_/2){
                        extension_progress_ago = 0;
                        Configuration intermediate_goal = this->getRandomConfiguration();
                        if(this->subsolver_ == SubsolverEnum::CASTAR){
                            int st = 0;
                            segment.clear();
                            for (st = 0; st < this->shake_trials_ && segment.size() < this->shake_length_; st++){
                                // std::cout << ANSI_BLUE << "\nShaking with CA*: " << st << "\n"  << ANSI_RESET;
                                Configuration intermediate_goal = this->getRandomConfiguration();
                                auto candidate_segment = this->computePrefix(suffix.back(), intermediate_goal, this->shake_length_);
                                if (candidate_segment.size() >= segment.size()){
                                    segment = candidate_segment;
                                }
                            }
                            // segment = this->computePrefix(suffix.back(), intermediate_goal, this->shake_length_);
                            for(auto c = segment.begin()+1; c != segment.end(); c++){
                                suffix.push_back(*c);
                            }
                            std::cout << ANSI_BLUE << "\nExtension Shaking with CASTAR: " << segment.size() << " steps\n" << ANSI_RESET;
                        } else if (this->subsolver_ == SubsolverEnum::DFS){
                            std::vector<std::shared_ptr<Configuration> > dfs_segment =
                                this->dfs_solver_.computeBoundedPathTowards(suffix.back(), intermediate_goal, this->shake_length_/2);
                            for(auto c = dfs_segment.begin()+1; c != dfs_segment.end(); c++){
                                suffix.push_back(**c);
                            }
                            std::cout << ANSI_BLUE << "\nExtension Shaking with DFS: " << dfs_segment.size() << " steps\n" << ANSI_RESET;
                        }
                    }
                }
            } else {
                bestSuffix = suffix;
                std::cout << ANSI_GREEN << "CA*-Shake solved the instance.\n" << ANSI_RESET;
                goal_reached = true;
                break;
            }
        }

        this->config_stack_ = prefix;
        for (auto it = bestSuffix.begin()+1; it != bestSuffix.end(); it++){
            this->config_stack_.push_back(*it);
        }


        // Check
        assert(this->config_stack_.front() == this->instance_.start());
        assert(this->config_stack_.back() == this->instance_.goal());
        for(auto c : this->config_stack_){
            this->instance_.graph().communication().isConfigurationConnected(c);
        }

        // Create the execution
        for (int i = 0; i < this->instance_.nb_agents(); i++)
        {
            std::shared_ptr<Path> path = std::make_shared<Path>();
            for (auto c : this->config_stack_)
            {
                path->push_back(c[i]);
            }
            // if (suffix.size() > 0){
            //     for (auto it = suffix.begin()+1; it != suffix.end(); it++)
            //     {
            //         path->push_back((*it)->at(i));
            //     }
            // }
            assert(path->isValid(this->instance_.graph().movement()));
            this->execution_.push_back(path);
        }
        return this->execution_;
    }
};
}