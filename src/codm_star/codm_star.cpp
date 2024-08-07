#include "codm_star.hpp"

using namespace codm_star;



ConnectedODMStar::ConnectedODMStar(
  std::shared_ptr<MovesGraph> movement_graph,
  std::shared_ptr<CommunicationsGraph> comm_graph,
  int subsolver_type,
  int optim_type,
  bool activate_logs,
  bool check_swapping_conflicts,
  double inflation_factor
) {
  movement_graph_ = movement_graph;
  comm_graph_ = comm_graph;
  activate_logs_ = activate_logs;
  check_swapping_conflicts_ = check_swapping_conflicts;
  inflation_factor_ = inflation_factor;
  subsolver_type_ = subsolver_type;
  optim_type_ = optim_type;
  recursive_call_ = false;

  if (subsolver_type_ == RECURSION) {
    planning_iter_ = 0;
    auto init_planners = std::unordered_map<std::vector<Agent>, std::shared_ptr<ConnectedODMStar>>();
    recursive_subplanners_ = std::make_shared<std::unordered_map<std::vector<Agent>, std::shared_ptr<ConnectedODMStar>>>(init_planners);
  }
}




ConnectedODMStar::ConnectedODMStar(
  std::vector<Agent> ma,
  ConnectedODMStar &parent
) {
  movement_graph_ = parent.movement_graph_;
  comm_graph_ = parent.comm_graph_;
  activate_logs_ = parent.activate_logs_;
  check_swapping_conflicts_ = parent.check_swapping_conflicts_;
  inflation_factor_ = parent.inflation_factor_;
  subsolver_type_ = parent.subsolver_type_;
  
  recursive_call_ = true;
  recursive_subplanners_ = parent.recursive_subplanners_;

  // init exploration
  explored_ = {};
  all_agents_ = {{}};
  agents_ = ma;
  nb_agents_ = ma.size();
  planning_iter_ = 0;
  policies_ = parent.policies_;
  
  for (Agent a = 0; a < nb_agents_; a++) {
    all_agents_.at(0).insert(a);
  }
}




void ConnectedODMStar::preprocess_agents_individual_paths(ODconfig t) {

  if (!recursive_call_) {
    explored_ = {};
    nb_agents_ = t.config.size();
    policies_ = {};
    agents_ = {};

    for (Agent a = 0; a < (Agent)nb_agents_; a++) {
      agents_.push_back(a);
      ShortestPaths current_policy = ShortestPaths(movement_graph_->get_adj_list(), a, t.config.at(a));
      policies_.push_back(std::make_shared<ShortestPaths>(current_policy));
    }

    all_agents_ = {std::set<Agent>(agents_.begin(), agents_.end())};
    
    if (subsolver_type_ == CCA_STAR) {
      subplanners_ = SubplannersManager(
        movement_graph_, 
        comm_graph_,
        policies_,
        activate_logs_,
        check_swapping_conflicts_
      );
    }
  }

}




std::shared_ptr<Node> ConnectedODMStar::get_node(
  ODconfig c
) {
  auto iterator = explored_.find(c);
  std::shared_ptr<Node> n;
  
  if(iterator != explored_.end()) {
    n = iterator->second;
    
    // check if ODagts have changed
    n->od_config.OD_ma_have_increased = n->od_config.OD_meta_agent != c.OD_meta_agent; 

    if (n->od_config.OD_ma_have_increased) {
      // If changed, replace ODagts (ODagts is contained in ODconfig c)
      n->od_config = c;
    }

  } else {
    std::shared_ptr<Node> new_node = std::make_shared<Node>(c);
    new_node->heuristic = get_heuristic(c);
    ODconfig config = c;
    explored_[config] = new_node;
    iterator = explored_.find(c);
    n = iterator->second;
  }

  if (recursive_call_) {
    // Reset the node if it's a new planning iteration
    n->reset(planning_iter_);
  }

  return n;
}




double ConnectedODMStar::get_heuristic(ODconfig c) {
  double h = 0;

  for (Agent a = 0; a < nb_agents_; a++) {
    if (c.assigned_moves.find(a) != c.assigned_moves.end()) {
      h += policies_.at(agents_.at(a))->get_cost(c.assigned_moves[a]);
    } else {
      h += policies_.at(agents_.at(a))->get_cost(c.config.at(a));
    }
  }

  return h * inflation_factor_;
}




double ConnectedODMStar::get_edge_cost(ODconfig s, ODconfig t) {
  double cost = 0;
  
  if (s.is_standard() && t.is_standard()) {
    
    for (Agent a = 0; a < nb_agents_; a++) {
      cost += policies_.at(agents_.at(a))->get_edge_cost(s.config.at(a), t.config.at(a));
    }

  } else if (s.config == t.config) {

    for (auto assigned_move_t: t.assigned_moves) {
      Agent a = assigned_move_t.first;

      if (s.assigned_moves.find(a) == s.assigned_moves.end()) {
        cost += policies_.at(agents_.at(a))->get_edge_cost(s.config.at(a), t.assigned_moves[a]);
      }
    }

  } else {

    for (Agent a = 0; a < nb_agents_; a++) {
      if (s.assigned_moves.find(a) == s.assigned_moves.end()) {
        cost += policies_.at(agents_.at(a))->get_edge_cost(s.config.at(a), t.config.at(a));
      }
    }

  }

  assert(cost >= 0);

  return cost;
}




Execution ConnectedODMStar::search(
  Configuration s, 
  Configuration t,
  uint64_t iterations_limit,
  uint32_t time_limit
) {

  assert(s.size() == t.size());

  ODconfig source = ODconfig(s);
  ODconfig target = ODconfig(t);

  preprocess_agents_individual_paths(target);

  const std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
  const std::chrono::time_point<std::chrono::system_clock> end = start + std::chrono::seconds(time_limit);

  Execution result;
  MetaAgentManager ma_manager;
  ConnectivityManager co_manager;
  nb_iterations = 0;

  std::shared_ptr<Node> first = get_node(source);
  first->best_predecessor = first;
  first->visited = false;
  first->cost = 0;

  OpenList open_list;
  open_list.push(first);

  while (open_list.size() > 0) {
    
    // Get the current node
    std::shared_ptr<Node> n = open_list.top();
    open_list.pop();

    // Check if the current node has been visited
    if (n->visited) {
      continue;
    }

    // ODrM* recursive call
    if (recursive_call_) { 
      if (n->od_config == t) {
        n->forwards_ptr = n;
      }

      if (n->forwards_ptr != nullptr) {
        return back_trace_path_init(n);
      }
    }

    // Halting conditions
    if (n->od_config.config == t || 
        std::chrono::system_clock::now() > end ||
        nb_iterations > iterations_limit) {
      return get_execution(n);
    }

    // Collection of the new meta-agents formed by the conflicts in potential successors.
    MetaAgents new_meta_agents;

    // Mark the current node as visited.
    n->visited = true;

    // Get all the potential successors of n.
    std::vector<ODconfig> successors = get_successors(n);
    size_t open_size_before_it = open_list.size();

    // "Type of conflict" initialization
    current_conflict_ = NO_CONFLICT;

    // Search iteration: compute the next successors
    for(ODconfig successor: successors) {
      visit_successor(successor, n, new_meta_agents, open_list, ma_manager, co_manager);
    }

    // Backpropagate conflict information to merge new meta-agents
    back_propagation(n, new_meta_agents, open_list);

    // Manage the case where the subsolver does not find a successor
    no_successor(open_size_before_it, n, open_list);

    if (activate_logs_) {
      CodmLogs codm_logs = CodmLogs();
      LOG_DEBUG("node visited: ");
      codm_logs.log_debug_od_config(n->od_config);
      LOG_DEBUG("iterations: " << nb_iterations);
      LOG_DEBUG("open list size: " << open_list.size());
    }

    nb_iterations++;
  }

  return result;
}




void ConnectedODMStar::visit_successor(
  ODconfig s,
  std::shared_ptr<Node> n,
  MetaAgents &new_meta_agents, 
  OpenList &open_list,
  MetaAgentManager &ma_manager,
  ConnectivityManager &co_manager
) {

  // Compute collision conflicts
  MetaAgents meta_agents_set = od_collisions(n->od_config, s);
  bool successor_has_collisions = meta_agents_set.size() > 0; 
  bool successor_is_disconnected = s.is_standard() && !comm_graph_->is_configuration_connected(s.config) && !recursive_call_;
  bool successor_has_conflict = successor_has_collisions || successor_is_disconnected;

  if (successor_has_conflict) {

    if (successor_has_collisions) {
      ma_manager.meta_agents_set_union(meta_agents_set, new_meta_agents);
      current_conflict_ = COLLISION;
    }

    if (successor_is_disconnected) {
      MetaAgents coupled_agents = all_agents_;
      ma_manager.meta_agents_set_union(coupled_agents, new_meta_agents);
      current_conflict_ = CONNECTIVITY;
    }

  } else { 

    // Else if no conflict is detected, get the successor of n
    std::shared_ptr<Node> successor = get_node(s);

    // Update the node data
    successor->predecessors.insert(n);
    MetaAgents prev_maset_successor = successor->meta_agents;
    ma_manager.meta_agents_set_union(prev_maset_successor, new_meta_agents);

    if (!successor->visited) {
      double successor_cost = n->cost + get_edge_cost(n->od_config, s);

      // Update the cost of the successor of n
      if (successor_cost < successor->cost) {

        // Update the successor meta-agents with the predecessor meta-agents
        if (successor->od_config.is_standard()) {
          ma_manager.meta_agents_set_union(n->meta_agents, successor->meta_agents);
        }

        successor->cost = successor_cost;
        successor->best_predecessor = n;

        assert(successor->cost >= n->cost);

        open_list.push(successor);

      }
    } else { 
      ma_manager.meta_agents_set_union(successor->meta_agents, new_meta_agents);
    }

    if (successor->od_config.OD_ma_have_increased) {
      successor->side_successor = true;      
    }
  }

}




void ConnectedODMStar::back_propagation(
  std::shared_ptr<Node> n, 
  MetaAgents new_meta_agents, 
  OpenList &open
) {

  MetaAgentManager ma_manager;

  if(ma_manager.meta_agents_set_union(new_meta_agents, n->meta_agents)) {

    if (n->visited) {
      n->visited = false;
      open.push(n);
    }

    for (std::shared_ptr<Node> p: n->predecessors) {
      back_propagation(p, n->meta_agents, open);
    }
  }
}




void ConnectedODMStar::no_successor(
  size_t open_size_before_it,
  std::shared_ptr<Node> n,
  OpenList &open
) {

  // If no valid successor is added to Open,
  // run back_propagation_ma_od to ensure the invariant
  // that n has potential predecessors inside Open.
  if (open_size_before_it == open.size()) {
    back_propagation_ma_od(n);
  }

  // If Open is empty, insert the "side successors"
  // into Open.  
  if (open.size() == 0) {
    for (auto& it: explored_) {
      if (it.second->side_successor) {
      
        it.second->side_successor = false;

        if (it.second->visited) {
         it.second->visited = false;
        }

        open.push(it.second);

        break;
      }

    }
  }

}




std::optional<std::shared_ptr<Node>> ConnectedODMStar::get_standard_node(
  ODconfig c
) {
  c = ODconfig(c.config);
  auto iterator = explored_.find(c);
  
  if(iterator != explored_.end()) {
    std::shared_ptr<Node> n = iterator->second;
    return n;
  }

  return std::nullopt;
}





void ConnectedODMStar::back_propagation_ma_od(std::shared_ptr<Node> n) {

  std::set<std::shared_ptr<Node>> visited_nodes;
  std::queue<std::shared_ptr<Node>> predecessors_queue;
  predecessors_queue.push(n);

  while (!predecessors_queue.empty()) {
    
    std::shared_ptr<Node> current_node = predecessors_queue.front();
    predecessors_queue.pop();
    visited_nodes.insert(current_node);

    MetaAgent full_OD_meta_agent; 
    auto assigned_moves = current_node->od_config.assigned_moves;

    // create the following collection: full_OD_meta_agent := Agt \ dom(c')
    for (Agent a = 0; a < nb_agents_; a++) {
      if (assigned_moves.find(a) == assigned_moves.end()) {
        full_OD_meta_agent.insert(a);
      }
    }

    bool node_was_in_open = !current_node->visited || current_node->side_successor;
    if (node_was_in_open) continue;


    if (current_node->od_config.OD_meta_agent != full_OD_meta_agent) {

      // The node does not respect n.ODagts = Agt \ dom(c')
      current_node->od_config.OD_meta_agent = full_OD_meta_agent;
    
      // The `side_successor` flag indicates that the node is re-openable.
      current_node->side_successor = true; 
    
    } else {

      for (std::shared_ptr<Node> p: current_node->predecessors) {
        // The only nodes that can have empty ODagts are the standard nodes. Only visit those nodes.
        std::optional<std::shared_ptr<Node>> standard_pred = get_standard_node(p->od_config);
        if (standard_pred.has_value() && visited_nodes.find(standard_pred.value()) == visited_nodes.end()) 
          predecessors_queue.push(standard_pred.value());
      }
    }

  }

}




std::vector<ODconfig> ConnectedODMStar::get_successors(
  std::shared_ptr<Node> n
) {

  if (n->od_config.is_OD_in_process()) {

    ////////////////////
    // Optimal solver //
    ////////////////////

    if (activate_logs_) {
      LOG_DEBUG("compute OD, size of OD_agents meta-agent: " << n->od_config.OD_meta_agent.size());
    }

    return compute_od(n);

  } else {
    
    ////////////////
    // Sub-solver //
    ////////////////

    if (activate_logs_) {
      LOG_DEBUG("compute subsolver, size of the configuration: " << n->od_config.config.size());
    }
    
    return compute_sub_solvers(n);
  }
}




std::vector<ODconfig> ConnectedODMStar::compute_od(std::shared_ptr<Node> n) {

  if (subsolver_type_ == CCA_STAR && optim_type_ != NO_OPTIM) {
    // OD cut optimization
    ODconfig rand_succ = get_random_successor(n);
    if (explored_.find(rand_succ) == explored_.end()) {
      return { rand_succ };
    }
  }

  return od_successors(n->od_config);
}




std::vector<ODconfig> ConnectedODMStar::od_successors(ODconfig c) {
  std::vector<ODconfig> successors;
  Agent a = c.next_agent_to_compute();
  AgentPosition p = c.config.at(a);
  bool successor_is_valid = false;

  auto neighbors = boost::adjacent_vertices(p, movement_graph_->get_adj_list());
  for (AgentPosition current_pos: make_iterator_range(neighbors)) {

    ODconfig successor;
    successor.config = c.config;
    successor.OD_meta_agent = c.OD_meta_agent;
    successor.OD_meta_agent.erase(a);
    successor.assigned_moves = c.assigned_moves;
    successor.OD_ma_have_increased = false;
    successor.generated_by_subsolver = false;

    // Assign the new position to this agent.
    successor.assigned_moves[a] = current_pos;
    successor.last_agent_assigned = a;
    successor_is_valid = true;

    if (successor.is_OD_finished()) {

      // If all moves have been assigned, update the moves in the config.
      for (auto it = successor.assigned_moves.begin(); it != successor.assigned_moves.end(); it++) {
        successor.config.at(it->first) = it->second;
      }
      
      // Clear the data related to OD.
      successor.assigned_moves.clear();
      successor.OD_meta_agent.clear();
      successor_is_valid = successor.config != c.config;
    }

    if (successor_is_valid) {
      successors.push_back(successor);
    }
  }

  return successors;
}




std::vector<ODconfig> ConnectedODMStar::compute_sub_solvers(std::shared_ptr<Node> n) {

  if (subsolver_type_ == RECURSION && n->meta_agents == all_agents_) {
    n->od_config.OD_meta_agent = all_agents_.at(0);
    return od_successors(n->od_config);
  }

  assert(n->od_config.is_standard());
  
  ODconfig successor = subsolver_successor(n);

  if (subsolver_type_ == CCA_STAR && successor.OD_meta_agent.size() != 0) {
    auto new_succ = get_next_config_from_subsolver(all_agents_.at(0), n->od_config.config);
    if (new_succ.has_value()) {
      return { new_succ.value() };
    }
  }

  return { successor };
}




ODconfig ConnectedODMStar::subsolver_successor(std::shared_ptr<Node> n) {

  ODconfig successor = n->od_config;
  successor.generated_by_subsolver = true;
  MetaAgent new_OD_agents;

  // Assign the individual optimal moves to singleton meta-agents.
  for (Agent a = 0; a < nb_agents_; a++) {
    AgentPosition current_position = n->od_config.config.at(a);
    successor.assigned_moves[a] = policies_.at(agents_.at(a))->get_step(current_position); 
  }

  // Assign subplanners' moves to meta-agents.
  for (MetaAgent ma: n->meta_agents) {

    // Get current configuration.
    Configuration ma_configuration;
    for (Agent a: ma) {
      ma_configuration.push_back(n->od_config.config.at(a));
    }

    // Call subsolver.
    std::optional<Configuration> ma_successor = get_next_config_from_subsolver(ma, ma_configuration);

    if (ma_successor.has_value()) { 
      // Subsolver success
      int i = 0;
      for (Agent a: ma) {
        successor.assigned_moves[a] = ma_successor.value().at(i);
        i++;
      }
    
    } else {                       
      // Subsolver fails
      new_OD_agents.insert(ma.begin(), ma.end());
    }
  }

  // Remove the assigned moves from subsolvers that have failed.
  for (const Agent a: new_OD_agents) {
    successor.assigned_moves.erase(a);
  }

  // If some subsolvers have failed, 
  // update the `OD_meta_agent` of the successor.
  successor.OD_meta_agent = new_OD_agents;
  successor.assign_moves();

  return successor;
}




std::optional<Configuration> ConnectedODMStar::get_next_config_from_subsolver(
  MetaAgent ma, 
  Configuration c
) {
  assert(c.size() > 0);

  Configuration s;
  Configuration t;
  std::vector<Agent> original_ma;
  std::shared_ptr<ConnectedODMStar> recursive_call;
  Configuration result;

  switch(subsolver_type_) {

    case CCA_STAR:
      // get the successor configuration from CA*
      s = subplanners_.get_successor(ma, c, false);
      if (s.size() > 0) { return s; } else { return std::nullopt; }
      break;

    case RECURSION:
      // Get the successor configuration from a recursive call to ODrM*.
      for (Agent a: ma) {
        // Recursive CODM* uses local agent IDs.
        // We need to convert these IDs into the original agent IDs
        // from the original CODM* call. This line is necessary for that:
        original_ma.push_back(agents_.at(a));
        
        // Get the target configuration from policies:
        t.push_back(policies_.at(agents_.at(a))->get_goal());
      }
      
      if (recursive_subplanners_->find(original_ma) == recursive_subplanners_->end()) {
        recursive_subplanners_->insert(
          { original_ma, std::make_shared<ConnectedODMStar>(ConnectedODMStar(original_ma, *this)) } );
      }

      recursive_call = recursive_subplanners_->at(original_ma);
      result = recursive_call->get_step(ODconfig(c), ODconfig(t));
      
      return result;
      break;

    case NAIVE:
      return std::nullopt;
      break;
  }

  return std::nullopt;
}




ODconfig ConnectedODMStar::get_random_successor(std::shared_ptr<Node> n) {

  // Successor expansion is incomplete, so this node may be potentially re-opened.
  n->side_successor = true;

  ODconfig successor = n->od_config;
  successor.generated_by_subsolver = true;
  MetaAgent ma = n->od_config.OD_meta_agent.size() > 0 ? n->od_config.OD_meta_agent : all_agents_.at(0);

  Configuration ma_configuration;
  for (Agent a: ma) {
    ma_configuration.push_back(n->od_config.config.at(a));
  }

  Configuration ma_successor = subplanners_.get_successor(ma, ma_configuration, true);
  bool success = ma_successor.size() > 0;

  if (success) {
    int i = 0;
    for (Agent a: ma) {
      successor.assigned_moves[a] = ma_successor.at(i);
      i++;
    }

    successor.assign_moves();
  }

  return successor;
}




MetaAgents ConnectedODMStar::od_collisions(ODconfig &s, ODconfig &t) {
  assert(s.config.size() == nb_agents_ && t.config.size() == nb_agents_);

  CollisionsManager coll_manager;  
  Configuration source = s.config;
  Configuration target = t.config;

  if (!t.is_standard()) {
    for (Agent a: t.OD_meta_agent) {
      if (t.assigned_moves.find(a) != t.assigned_moves.end()) { 
        target[a] = t.assigned_moves[a]; 
      } else {
        target[a] = -1;
      }
    }
  }

  if (check_swapping_conflicts_) {
    return coll_manager.get_coupled_agent_swap_collision(source, target); 
  }

  return coll_manager.get_coupled_agent(source, target);
}




bool ConnectedODMStar::check_config_validity(
  Configuration pred_c, Configuration c
) {

  bool valid_move = false;

  for (AgentPosition a = 0; a < nb_agents_; a++) {
    // Check if the current position is contained in the
    // neighborhood of the previous position.
    AgentPosition pred_pos = pred_c.at(a);
    AgentPosition current_pos = c.at(a);

    auto neighbors = boost::adjacent_vertices(pred_pos, movement_graph_->get_adj_list());
    valid_move = false;

    for (AgentPosition p: make_iterator_range(neighbors)) {
      if (current_pos == p) {
        valid_move = true;
        break;
      }
    }
  }

  ODconfig predecessor = ODconfig(pred_c);
  ODconfig successor = ODconfig(c);
  bool not_colliding = (od_collisions(predecessor, successor).size() == 0);
  bool connected = comm_graph_->is_configuration_connected(successor.config);

  return not_colliding && valid_move && connected;
}




Execution ConnectedODMStar::get_execution(
  std::shared_ptr<Node> n
) {

  Execution result;
  std::shared_ptr<Node> current = n;
  Configuration current_config = current->od_config.config;
  Configuration pred_config = current_config;
  
  while (current->best_predecessor != current) {
    assert(check_config_validity(pred_config, current_config));

    if (pred_config != current_config || result.size() == 0) {
      result.insert(result.begin(), current_config);
    }

    pred_config = current_config;
    current = current->best_predecessor;
    current_config = current->od_config.config;
  }
  result.insert(result.begin(), current->od_config.config);
  
  return result;
}



/////////////////////////
// CODM* optimizations //
/////////////////////////




void ConnectedODMStar::preprocess_topological_graph() {
  const std::map<AgentPosition, std::vector<float>> nodes = movement_graph_->get_positions();
  const int nb_expands = 7;

  for (const auto& [node, position]: nodes) {

    std::unordered_set<AgentPosition> neighbors_set({node});
    for (int i = 0; i < nb_expands; i++) {
      for (AgentPosition neighbor: neighbors_set) {
        std::unordered_set<AgentPosition> new_neighbors = movement_graph_->get_neighbors_set(neighbor);
        neighbors_set.insert(new_neighbors.begin(), new_neighbors.end());
      }
    }

    positions_score_.push_back(neighbors_set.size());
  }

}





uint64_t ConnectedODMStar::get_configuration_score(
  Configuration c
) {
  uint64_t score = 0;

  for (AgentPosition p: c) {
    score += positions_score_.at(p);
  }

  return score;
}




Execution ConnectedODMStar::bidirectional_search(
  Configuration s, 
  Configuration t,
  std::optional<std::pair<uint64_t, double>> iterations_parameters,
  std::optional<std::pair<uint32_t, double>> time_parameters,
  bool activate_score
) {
  
  // Compute the degree of liberty for each position.
  preprocess_topological_graph(); 

  Execution exec_forward = {s};
  Execution exec_backward = {t};
  Execution current_exec = {};

  std::pair<uint64_t, double> iterations_pair = iterations_parameters.value_or(
    std::pair<uint64_t, double>(std::numeric_limits<uint64_t>::max(), 1)
  );
  std::pair<uint32_t, double> time_pair = time_parameters.value_or(
    std::pair<uint32_t, double>(std::numeric_limits<uint32_t>::max(), 1)
  );

  uint64_t source_score = get_configuration_score(s);
  uint64_t target_score = get_configuration_score(t);

  uint64_t iterations_limit = iterations_pair.first;
  uint32_t time_limit = time_pair.first;

  bool is_forward = true; 
  if (activate_score) {
    is_forward = (source_score <= target_score);
  }

  bool is_goal_reached = (s == t);

  while(!is_goal_reached) {

    Configuration source = exec_forward.back();
    Configuration target = exec_backward.back(); 

    if (activate_logs_) {
      LOG_DEBUG("execution size: " << current_exec.size());
      LOG_DEBUG("is forward: " << is_forward);

      LOG_DEBUG("source score: " << get_configuration_score(source) << 
                "; target score: " << get_configuration_score(target));
    }

    current_exec = {};
    
    // Alternation between forward and backward search.
    if (is_forward) {
      current_exec = search(source, target, iterations_limit, time_limit);
    } else {
      current_exec = search(target, source, iterations_limit, time_limit);
    }

    // Add the current local execution to the global ones.
    if (current_exec.size() > 1) {
      if (is_forward) {
        exec_forward.insert(exec_forward.end(), current_exec.begin() + 1, current_exec.end());
      } else {
        exec_backward.insert(exec_backward.end(), current_exec.begin() + 1, current_exec.end());
      }
    }

    is_goal_reached = (exec_forward.back() == exec_backward.back());

    if (activate_score) {
      source_score = get_configuration_score(exec_forward.back());
      target_score = get_configuration_score(exec_backward.back());
      is_forward = (source_score <= target_score);
    } else {
      is_forward = !is_forward;
    }

    is_forward = !is_forward;  

    // Update the time and the number of iteration limits.
    iterations_limit = iterations_limit * iterations_pair.second;
    time_limit = time_limit * time_pair.second;
  }

  exec_forward.insert(exec_forward.end(), exec_backward.rbegin() + 1, exec_backward.rend());

  assert(exec_forward.at(0) == s && exec_forward.back() == t);
  for (int i = 0; i < exec_forward.size() - 1; i++) {
    assert(check_config_validity(exec_forward.at(i), exec_forward.at(i+1)));
  }

  return exec_forward;
}




/////////////////////////
// ODrM* optimizations //
/////////////////////////




Configuration ConnectedODMStar::get_node_step(std::shared_ptr<Node> n) {
  assert(n != nullptr);

  while (!n->forwards_ptr->od_config.is_standard()){
    n = n->forwards_ptr;
    assert(n != nullptr);
  }

  return n->forwards_ptr->od_config.config;  
}




Configuration ConnectedODMStar::get_step(ODconfig s, ODconfig t) {
  
  std::shared_ptr<Node> n = get_node(s);
  if (n->forwards_ptr != nullptr){
    return get_node_step(n);
  }

  search(s.config, t.config);
  planning_iter_++;

  return get_node_step(n);
}




Execution ConnectedODMStar::back_trace_path_init(std::shared_ptr<Node> n) {
  back_trace_path(n, n->forwards_ptr);
  return {};
}




void ConnectedODMStar::back_trace_path(std::shared_ptr<Node> n, std::shared_ptr<Node> s) {
  n->forwards_ptr = s;

  if (n != s) {
    n->heuristic = s->heuristic + get_edge_cost(n->od_config, s->od_config);
  } else {
    n->heuristic = 0;
  }

  if (n->best_predecessor != n) {
    assert(n->best_predecessor != nullptr);
    assert(n->cost >= n->best_predecessor->cost);

    back_trace_path(n->best_predecessor, n);
  }
}