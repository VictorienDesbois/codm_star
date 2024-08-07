#include "cca_star.hpp"

using namespace cca_star;


// constructors
CCAstar::CCAstar() { 
  check_swapping_conflicts_ = false; 
}




CCAstar::CCAstar(
  std::shared_ptr<MovesGraph> movement_graph,
  std::shared_ptr<CommunicationsGraph> comm_graph,
  std::vector<std::shared_ptr<ShortestPaths>> policies,
  bool check_swapping_conflicts
) {
  movement_graph_ = movement_graph;
  comm_graph_ = comm_graph;
  all_policies_ = policies;
  check_swapping_conflicts_ = check_swapping_conflicts;
  heuristic_type_ = SHORTESTPATH;
  shuffle_type_ = MERGED;
}




CCAstar::~CCAstar() {}



void CCAstar::set_heuristic_type(int h_type) {
  assert(h_type >= SHORTESTPATH && h_type <= BIRDEYE);
  heuristic_type_ = h_type;
}




void CCAstar::set_shuffle_type(int shuffle_type) {
  assert(shuffle_type >= SEPARATE && shuffle_type <= MERGED);
  shuffle_type_ = shuffle_type;
}




Execution CCAstar::search(Configuration s, Configuration t, MetaAgent ma) {
  assert(comm_graph_->is_configuration_connected(s));
  
  for (Agent a: ma) {
    policies_.push_back(all_policies_.at(a));    
  }
  Execution result = compute_execution(s, t);    
  policies_.clear();

  return result;
}




void CCAstar::deterministic_random_shuffle(std::vector<Agent> &agent_vector) {

  std::vector<Agent>::iterator first = agent_vector.begin();
  auto last = agent_vector.end();

  typedef typename std::iterator_traits<std::vector<Agent>::iterator>::difference_type diff_t;

  for (diff_t i = last - first - 1; i > 0; --i)
  {
    using std::swap;
    swap(first[i], first[std::rand() % (i + 1)]);
  }
}




void CCAstar::standard_random_shuffle(std::vector<Agent> &agent_vector) {
  std::random_device rd;
  std::mt19937 g(rd()); 

  std::shuffle(agent_vector.begin(), agent_vector.end(), g);
}




std::vector<Agent> CCAstar::shuffle_agents(uint nb_agents, Configuration s, Configuration t) {
  // Give a priority order to agents using random shuffle.

  std::vector<Agent> priority_order;
  std::vector<Agent> goal_agents;
  float distance;

  switch(shuffle_type_) {
  
    case SEPARATE:
      for (Agent a = 0; a < nb_agents; a++) {
        distance = movement_graph_->euclidean_distance(s.at(a), t.at(a));

        if (distance != 0.0) {
          priority_order.push_back(a);
        } else {
          goal_agents.push_back(a);
        }
      }
      
      standard_random_shuffle(goal_agents);
      standard_random_shuffle(priority_order);
      priority_order.insert(priority_order.end(), goal_agents.begin(), goal_agents.end());
      break;

    case MERGED:
      for (Agent a = 0; a < nb_agents; a++) {
        priority_order.push_back(a);
      }
      
      deterministic_random_shuffle(priority_order);   
      break;
  
  }

  return priority_order;
}




std::unordered_set<AgentPosition> CCAstar::init_cluster(const Configuration &start, std::vector<Agent> &shuffled, uint nb_agents) {
  Agent first_agent = shuffled.at(0);

  std::unordered_set<AgentPosition> cluster = comm_graph_->get_neighbors_set(start.at(first_agent));

  for (int a = 1; a < nb_agents; a++) {
    Agent a_to_switch = a;

    // While the cluster does not contain the position of the current agent `a`.
    while(cluster.count(start.at(shuffled.at(a_to_switch))) != 1 && a_to_switch < (nb_agents - 1)) {
      a_to_switch++;
    }

    assert(cluster.count(start.at(shuffled.at(a_to_switch))) == 1);
    Agent tmp = shuffled.at(a);
    shuffled[a] = shuffled.at(a_to_switch);
    shuffled.at(a_to_switch) = tmp;

    auto agent_neighbors_pos = comm_graph_->get_neighbors_set(start.at(shuffled[a]));

    for (auto pos: agent_neighbors_pos) {
      cluster.insert(pos);
    }
  }

  // Safety check.
  cluster = comm_graph_->get_neighbors_set(start.at(first_agent));
  for(Agent i = 1; i < nb_agents; i++){
    assert(cluster.count(start.at(shuffled[i])) == 1);
    for(auto pos: comm_graph_->get_neighbors_set(start.at(shuffled.at(i)))) {
      cluster.insert(pos);
    }
  }

  return cluster;
}




Path CCAstar::get_shortest_path(Agent a, AgentPosition s, AgentPosition t) {
  Path result;

  switch(heuristic_type_) {

    case SHORTESTPATH:
      result = policies_.at(a)->get_shortest_path(s);
      break; 

    case BIRDEYE:
      result = ShortestPaths(movement_graph_->get_adj_list(), a, t).get_shortest_path(s); 
      break;

  } 

  return result;
}




void CCAstar::extend_neighborhoods(
  std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
  std::vector<Path> &paths,
  int &path_size
) {

  // If the added path is longer than the previous ones, extend the neighborhoods.
  
  path_size = paths.back().size();

  for (int t = neighborhoods.size(); t < path_size; t++) {
    neighborhoods.push_back(neighborhoods.back());
  }

  for (int i = 0; i < paths.size(); i++) {
    if (paths.at(i).size() < path_size) {
      for (int t = paths.at(i).size(); t < path_size; t++) {
        paths.at(i).push_back(paths.at(i).back());
      }
    }
  }

}




void CCAstar::shorten_neighborhoods(
  std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
  std::vector<Path> &paths,
  int &path_size
) {

  // If the added path is shorter, shorten all previous paths and neighborhoods.

  path_size = paths.back().size();
  
  neighborhoods.resize(path_size);

  for(int i = 0; i < paths.size(); i++) {
    paths.at(i).resize(path_size);
  }
}




void CCAstar::constrain_neighborhoods(
  std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
  std::vector<Path> &paths,
  Agent current_agent
) {
  
  // Modify neighborhood for the next path computation.
  for (int t = 0; t < neighborhoods.size(); t++) {
    // Add the neighborhood in communication to the new path.
    int time_step = t < paths.back().size() ? t : (paths.back().size() - 1);

    for (auto pos: comm_graph_->get_neighbors_set(paths.back().at(time_step))) {
      // Add the neighboring connected positions of the current agent to the neighborhood.
      neighborhoods.at(t).insert(pos);
    }

    for (Agent i = 0; i <= current_agent; i++){
      time_step = t < paths.at(i).size() ? t : (paths.at(i).size() - 1);
      // Erase the position occupied by the agent during timestep i.
      neighborhoods.at(t).erase(paths.at(i).at(time_step));
    }
  }
}




Execution CCAstar::compute_execution(
  const Configuration& start, 
  const Configuration& goal, 
  size_t max_path_size
) {
  uint nb_agents = start.size();
  max_path_size = -1;

  // Randomize the agents' priority order.
  std::vector<Agent> shuffled = shuffle_agents(nb_agents, start, goal);

  // Initialize the neighborhood of the first agent.
  std::unordered_set<AgentPosition> cluster = init_cluster(start, shuffled, nb_agents);

  // Compute the first path.
  std::vector<Path> paths;
  paths.push_back(get_shortest_path(shuffled.at(0), start.at(shuffled.at(0)), goal.at(shuffled.at(0))));

  if (max_path_size > 0 && paths.back().size() > max_path_size) {
    paths.back().resize(max_path_size);
  }

  int path_size = paths.back().size();

  // Initialize neighborhoods: neighborhoods[t] is the set of unoccupied positions that communicate with some positions occupied by
  // some agent at time t. Any subsequent agent must be and can be somewhere in neighboorhoods[t].
  std::vector<std::unordered_set<AgentPosition>> neighborhoods(paths.back().size());
  
  for (size_t t = 0; t < paths.back().size(); t++) {
    for (auto pos: comm_graph_->get_neighbors_set(paths.back().at(t))) {
      neighborhoods.at(t).insert(pos);
    }
    // Erase visited positions to avoid collisions.
    neighborhoods.at(t).erase(paths.back().at(t));
  }

  // Compute the path for each agent.
  for (Agent a = 1; a < nb_agents; a++) {
    AgentPosition a_in_shuffled = shuffled.at(a);
    Path current_path = compute_path(
      a_in_shuffled, start.at(a_in_shuffled), goal.at(a_in_shuffled), neighborhoods, paths
    );
    paths.push_back(current_path);

    if (max_path_size > 0 && paths.back().size() > max_path_size) {
      paths.back().resize(max_path_size);
    }

    if (paths.back().size() > path_size) {
      extend_neighborhoods(neighborhoods, paths, path_size);
    } else {
      shorten_neighborhoods(neighborhoods, paths, path_size);
    }

    constrain_neighborhoods(neighborhoods, paths, a);
  }

  assert(paths.size() == nb_agents);

  return execution_cleaner(path_size, nb_agents, paths, shuffled);
}




size_t CCAstar::get_heuristic(Agent a, AgentPosition s, AgentPosition t) {
  
  size_t heuristic_cost = 0;
  std::shared_ptr<ShortestPaths> agent_policy;

  switch(heuristic_type_) {
    
    case SHORTESTPATH:
      agent_policy = policies_.at(a);  
      heuristic_cost = agent_policy->get_cost(s);
      break;

    case BIRDEYE:
      heuristic_cost = comm_graph_->euclidean_distance(s, t);
      break;
  
  }

  return heuristic_cost;
}




Execution CCAstar::execution_cleaner(
  int path_size, 
  int nb_agents, 
  std::vector<Path> &paths, 
  std::vector<Agent> &shuffled
) {
  Execution result;

  for (int t = 0; t < path_size; t++) {
    Configuration c(nb_agents);

    for (int a = 0; a < nb_agents; a++) {
      int time_step = t < paths.at(a).size() ? t : (paths.at(a).size() - 1);
      c.at(shuffled.at(a)) = paths.at(a).at(time_step);
    }

    bool c_has_collision = std::adjacent_find(c.begin(), c.end()) != c.end();
    assert(!c_has_collision);
    result.push_back(c);
  }

  return result;
}




bool CCAstar::is_position_a_swapping_conflict(
  AgentPosition previous_p,
  AgentPosition current_p,
  std::vector<Path> &paths,
  size_t time_step
) {

  if (!check_swapping_conflicts_) { return false; }

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




void CCAstar::generate_potential_path(
  Agent agent,
  std::vector<std::vector<CAnode>> &prefix_h,
  std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
  std::vector<Path> &paths,
  AgentPosition goal  
) {

  for (size_t i = neighborhoods.size() - 1; i > 0; i--) {
    for (AgentPosition pos: neighborhoods.at(i-1)) {    
      for (AgentPosition succ_pos: movement_graph_->get_neighbors_set(pos)) {

        if (neighborhoods.at(i).count(succ_pos) != 1 ||
            is_position_a_swapping_conflict(pos, succ_pos, paths, i)) continue;
        
        CAnode succ_ca_node = prefix_h.at(i).at(succ_pos);
        CAnode candidate = CAnode(pos, succ_pos, succ_ca_node.has_path, succ_ca_node.suffix_length+1, succ_ca_node.h+1);

        if (pos == succ_pos) {
          CAnode candidate = CAnode(pos, succ_pos, succ_ca_node.has_path, succ_ca_node.suffix_length, succ_ca_node.h);
        }

        if (!prefix_h.at(i-1).at(pos).defined() || candidate < prefix_h.at(i-1).at(pos)) {
          prefix_h.at(i-1).at(pos) = candidate;
        }
      }

      if (!prefix_h.at(i-1).at(pos).defined()) {
        prefix_h[i-1][pos] = CAnode(
          pos, pos, false, 0, get_heuristic(agent, pos, goal), true
        );
      }
    }
  }

}




Path CCAstar::compute_path(
  Agent agent,
  AgentPosition start,
  AgentPosition goal,
  std::vector<std::unordered_set<AgentPosition>> &neighborhoods,
  std::vector<Path> &paths
) {

  assert(neighborhoods.at(0).count(start) == 1);
  
  Path path;

  const int num_of_nodes = movement_graph_->node_count(); 

  std::vector<AgentPosition> suffix_next;
  std::vector<size_t> suffix_h;

  std::vector<bool> has_path_to_goal;
  std::vector<AgentPosition> has_path_to_goal_support;
  
  std::stack<AgentPosition> wait;

  for(int i = 0; i < num_of_nodes; i++) {
    suffix_next.push_back(std::numeric_limits<AgentPosition>::max());
    suffix_h.push_back(std::numeric_limits<size_t>::max());
    has_path_to_goal.push_back(false);
  }

  suffix_next.at(goal) = goal;
  suffix_h.at(goal) = 0;

  const int last_neighbors = neighborhoods.size() - 1;

  if (neighborhoods.at(last_neighbors).count(goal) == 1) {
    wait.push(goal);
  }

  while(!wait.empty()) {
    AgentPosition current = wait.top();
    wait.pop();

    if (has_path_to_goal.at(current)) continue;

    has_path_to_goal.at(current) = true;
    has_path_to_goal_support.push_back(current);

    for (AgentPosition succ: movement_graph_->get_neighbors_set(current)) {
      if (neighborhoods[last_neighbors].count(succ) == 1 && !has_path_to_goal.at(succ)) {
        suffix_next.at(succ) = current;
        suffix_h.at(succ) = suffix_h.at(current) + 1;
        wait.push(succ);
      }
    }
  }

  std::vector<std::vector<CAnode>> prefix_h;

  for (size_t i = 0; i < neighborhoods.size(); i++) {
    prefix_h.push_back(std::vector<CAnode>(num_of_nodes));
  }

  for (AgentPosition pos: has_path_to_goal_support) {
    prefix_h.at(last_neighbors).at(pos) = CAnode(
      pos, pos, true, suffix_h.at(pos), suffix_h.at(pos)
    );
  }

  for (AgentPosition pos: neighborhoods.at(last_neighbors)) {
    if (!has_path_to_goal.at(pos)) {
      prefix_h.at(last_neighbors).at(pos) = CAnode(
        pos, pos, false, 0, get_heuristic(agent, pos, goal)
      );
    }
  }

  generate_potential_path(agent, prefix_h, neighborhoods, paths, goal);

  assert(prefix_h.at(0).at(start).defined());
  CAnode ca_node = prefix_h.at(0).at(start);
  path.push_back(ca_node.node);

  for(size_t i = 1; !ca_node.dead_end && i < neighborhoods.size(); i++) {
    assert(prefix_h.at(i).at(ca_node.successor).defined());
    ca_node = prefix_h.at(i).at(ca_node.successor);
    path.push_back(ca_node.node);
  }

  if (ca_node.has_path) {
    AgentPosition pos = ca_node.successor;
    while (pos != goal) {
      pos = suffix_next.at(pos);
      path.push_back(pos);
    }
  }

  return path;
}
