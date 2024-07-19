#define LOG_DEBUG(s) std::cerr << "DEBUG: " << s << "\n"
#define LOG_INFO(s) std::cerr << "INFO: " << s << "\n"
#define LOG_WARNING(s) std::cerr << "WARNING: " << s << "\n"

#ifndef CMAPF_TYPES
#define CMAPF_TYPES

#pragma once
#include <cmath>
#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/dynamic_bitset.hpp>


/**************************************************************************
 * Provides type defs that are used in multiple files
 *************************************************************************/

namespace cmapf{

  typedef uint64_t Agent;
  typedef std::set<Agent> Agents;
  typedef int AgentPosition;
  typedef std::vector<AgentPosition> Path;

  struct Configuration: std::vector<AgentPosition> {
    Configuration(): std::vector<AgentPosition>(){};
    Configuration(size_t size): std::vector<AgentPosition>(size){};
    Configuration(size_t size, AgentPosition p): std::vector<AgentPosition>(size, p){};    

    bool operator==(Configuration const &other) const {
      if (this->size() != other.size()) { return false; }
      const size_t size = this->size();
      for (size_t i = 0; i < size; i++) {
        if (this->at(i) != other.at(i)) { return false; }
      }
      return true;
    }
  };

  typedef std::vector<Configuration> Execution;

  typedef boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::bidirectionalS,
    boost::no_property,
    boost::property<boost::edge_weight_t, double>
  > Moves;

  typedef boost::adjacency_list<
    boost::vecS, 
    boost::vecS,
    boost::bidirectionalS
  > Communications;
}

namespace std{
  // MetaAgent and Agents hash
  template <> 
  struct hash<std::set<uint64_t>>{
    size_t operator()(const std::set<uint64_t> &val) const{
      size_t hash = boost::hash_range(val.cbegin(), val.cend());
      return hash;
    }
  };

  // graph position
  template <> 
  struct hash<std::vector<float>>{
    size_t operator()(const std::vector<float> &val) const{
      size_t hash = boost::hash_range(val.cbegin(), val.cend());
      return hash;
    }
  };
}

#endif