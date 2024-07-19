#include "subplanners_manager.hpp"

using namespace codm_star;




SubplannersManager::SubplannersManager(
  std::shared_ptr<MovesGraph> movement_graph,
  std::shared_ptr<CommunicationsGraph> comm_graph,
  std::vector<std::shared_ptr<ShortestPaths>> policies,
  bool activate_logs,
  bool check_swapping_conflicts
) {
  movement_graph_ = movement_graph;
  comm_graph_ = comm_graph;
  policies_ = policies;
  nb_agents_ = policies.size();
  solver_ = cca_star::CCAstar(movement_graph_, comm_graph_, policies_, check_swapping_conflicts);  
  check_swapping_conflicts_ = check_swapping_conflicts;
  activate_logs_ = activate_logs;

  for (Agent a = 0; a < nb_agents_ ; a++) {
    target_.push_back(policies_.at(a)->get_goal());
  }
}




std::optional<Configuration> SubplannersManager::try_to_connect_configuration(Configuration c) {
  if(comm_graph_->is_configuration_connected(c)) {
    return c;
  }
  
  Configuration next_step = c;

  // try to create a connected source configuration
  for (Agent a = 0; a < c.size() && !comm_graph_->is_configuration_connected(next_step); a++) {
    AgentPosition optimal_step = policies_.at(a)->get_step(c.at(a));

    // verify if optimal_step does not generate a collision
    if (std::find(next_step.begin(), next_step.end(), optimal_step) == next_step.end()) {
      next_step.at(a) = optimal_step;
    }
  }

  if(comm_graph_->is_configuration_connected(next_step)) {
    return next_step;
  }

  return std::nullopt;
}




Configuration SubplannersManager::get_random_config(size_t nb_agents) {
  AgentPosition rand_pos = rand() % movement_graph_->node_count();
  Configuration result(nb_agents, rand_pos);
  return result;
}




Execution SubplannersManager::compute_cca_star(
  Configuration s, 
  Configuration t,
  MetaAgent ma,
  uint64_t nb_retry, 
  uint64_t opt) 
{
  Execution result = {};

  if (!comm_graph_->is_configuration_connected(s)) {
    std::optional<Configuration> conn_s = try_to_connect_configuration(s);
    if (conn_s == std::nullopt) {
      return {};
    } else {
      result.push_back(s);
      s = conn_s.value();
    }
  }

  switch(opt) {
    case CLASSIC:
      solver_.set_heuristic_type(0); // SHORTESTPATH
      break;
    case RANDOM: 
      t = get_random_config(ma.size());
      solver_.set_heuristic_type(1); // BIRDEYE
      break;
    default: 
      return {}; 
      break;
  }

  Execution try_result;

  for (int i = 0; i < nb_retry; i++) {
    if (i > nb_retry / 2) {
      t = get_random_config(ma.size());
      solver_.set_heuristic_type(1); // BIRDEYE
    }

    try_result = solver_.search(s, t, ma);

    if (try_result.size() > 1) { // success: the execution is not empty
      result.insert(result.end(), try_result.begin(), try_result.end());
      return result;
    }
  }

  // fail: the subsolver does not solve the instance
  return {}; 
}




Configuration SubplannersManager::get_successor(MetaAgent ma, Configuration c, bool rand) {
  assert(c.size() > 0);
  assert(c.size() == ma.size());

  Configuration successor;
  CollisionsManager coll_manager;

 // init
  Configuration current_source = c;
  Configuration current_target;
  for (Agent a: ma) {
    current_target.push_back(target_.at(a));
  }

  uint64_t nb_retry = ma.size();

  // if the configuration is not stored, then compute the execution associated with it
  if (!configuration_explored(ma, c) || rand) {

    int opt = (!rand) ? CLASSIC : RANDOM;

    // run the subsolver
    Execution ma_execution = compute_cca_star(current_source, current_target, ma, nb_retry, opt);
    subsolver_call++;

    // check if the subsolver return a valid execution 
    if (is_execution_valid(ma_execution)) {

      if (executions_storage_.find(ma) == executions_storage_.end()) {
        SubplannersManager::ExploredExecutions ma_explored_executions;
        executions_storage_[ma] = ma_explored_executions;
      }

      for (size_t i = 0; i < ma_execution.size(); i++) {
        Configuration current = ma_execution.at(i);
        if (i < ma_execution.size() - 1) {     
          Configuration current_successor = ma_execution.at(i+1);
          // if the execution is valid, store each configuration inside the map
          // that link meta-agent to the current execution
          executions_storage_[ma][current] = current_successor;
        } else {
          // manage the case of CA* execution of size one
          executions_storage_[ma][current] = current;
        }
      }

      subsolver_success_count++;
    }
    
    if (activate_logs_) {
      LOG_DEBUG("size of meta-agent: " << ma.size() <<  
                ", size of execution: "<< ma_execution.size() << ", " <<
                subsolver_success_count << " success / " << subsolver_call << " try");
    }
  }

  if (configuration_explored(ma, c)) {

    Configuration successor_configuration = executions_storage_.at(ma).at(c);
    if (config_already_visited_.find(successor_configuration) != config_already_visited_.end() 
        && !rand) {
      successor_configuration = get_successor(ma, c, true);
    }

    config_already_visited_.insert(successor_configuration);
    return successor_configuration; 
  
  } else {
    return {};
  }
}




bool SubplannersManager::is_configuration_valid(size_t nb_agents, Configuration pred_c, Configuration c) {

  CollisionsManager coll_manager;

  // check if the configuration does not contains collision
  bool not_colliding = true;

  if (check_swapping_conflicts_) {
    not_colliding = (coll_manager.get_coupled_agent_swap_collision(pred_c, c).size() == 0);
  } else {
    not_colliding = (coll_manager.get_coupled_agent(pred_c, c).size() == 0);
  }

  // check if the move is possible
  bool valid_move = true;

  for (Agent a = 0; a < nb_agents; a++) {
    AgentPosition pred_pos = pred_c.at(a);
    AgentPosition current_pos = c.at(a);

    auto neighbors = boost::adjacent_vertices(pred_pos, movement_graph_->get_adj_list());
    bool pos_found = false;

    for (AgentPosition p: boost::make_iterator_range(neighbors)) {
      if (current_pos == p) {
        pos_found = true;
        break;
      }
    }

    valid_move = valid_move && pos_found;
  }

  return valid_move && not_colliding; 
}




bool SubplannersManager::is_execution_valid(Execution e) {

  if (e.size() <= 0) { return false; }

  bool valid_exec = true;
  Configuration pred_c;
  
  // init
  pred_c = e.front();
  const int nb_agents = pred_c.size(); 

  for (Configuration c: e) {
    // check collision
    std::vector<bool> non_assigned(nb_agents, false);

    valid_exec = valid_exec && is_configuration_valid(nb_agents, pred_c, c);
    if (!valid_exec) { return false; }

    pred_c = c;
  }

  return valid_exec && e.size() > 1;
}




bool SubplannersManager::configuration_explored(MetaAgent ma, Configuration c) {
  auto storage_it = executions_storage_.find(ma);
  Configuration ma_target;

  for (Agent a: ma) {
    ma_target.push_back(target_.at(a));
  }

  if (storage_it != executions_storage_.end()) {
    ExploredExecutions explored_configs = storage_it->second;
    auto config_stored = explored_configs.find(c);

    if (config_stored != explored_configs.end()) {
      if (config_stored->first == config_stored->second) {
        return config_stored->first == ma_target;
      }

      return true;
    }
  }

  return false;
}




void SubplannersManager::print_stats(int nb_nodes_mainplanner) {
  int nb_nodes = 0; 
  
  for (auto it = executions_storage_.begin(); it != executions_storage_.end(); it++) {
    nb_nodes += it->second.size();
  }

  if (activate_logs_) {
    LOG_DEBUG("Number of subsolvers: " << executions_storage_.size() << "\n"
           << "Success rate of CA*: " << subsolver_success_count << "/" << subsolver_call << "\n"
           << "Number of nodes explored: " << (nb_nodes + nb_nodes_mainplanner) << "\n" << std::flush);
  }
}