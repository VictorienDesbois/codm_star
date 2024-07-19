#pragma once

#include "cmapf_types.hpp"


namespace cmapf{

  typedef std::set<Agent> MetaAgent;
  typedef std::vector<MetaAgent> MetaAgents;

  struct MetaAgentManager {
    MetaAgentManager() {}

    /**
     * @brief Check if the first set of agents is included in the second set.
     * 
     * @param contained The MetaAgent representing the subset.
     * @param container The MetaAgent representing the superset.
     * @return true if the first set is a subset of the second, false otherwise.
     */
    bool is_superset(MetaAgent contained, MetaAgent container) {
      bool is_contained = false;

      for (Agent a1: container) {
        is_contained = false;
        
        for (Agent a2: contained) {
          if (a1 == a2) {
            is_contained = true;
            break;
          }
        }

        if (!is_contained) {
          return false;
        } 
      }

      return true;
    }


    /**
     * @brief Check if two meta-agent sets are disjoint.
     * 
     * @param s1 The first MetaAgent set.
     * @param s2 The second MetaAgent set.
     * @return true if the two meta-agent sets are disjoint, false otherwise.
     */
    bool is_disjoint(const MetaAgent &s1, const MetaAgent &s2){

      for (auto i = s1.cbegin(); i != s1.cend(); ++i){
        for (auto j = s2.cbegin(); j != s2.cend(); ++j){
          if (*i == *j){
            return false;
          }
        }
      }

      return true;
    }


    /**
     * @brief Return the union of two sets of disjoint meta-agents.
     * 
     * @param c1 The first MetaAgents set.
     * @param c2 The second MetaAgents set.
     * @return true if the union of the two sets is successful, false otherwise.
     */
    bool meta_agents_set_union(MetaAgents &c1, MetaAgents &c2) {

      bool changed = false;

      while ((int)c1.size() > 0){

        int i = 0;
        bool found_overlap = false;

        while (i < (int)c2.size()){
          if (!is_disjoint(c2[i], c1.back())) {
            
            if (is_superset(c2[i], c1.back())){
              c1.pop_back();
              found_overlap = true;
              break;
            }

            c1.back().insert(c2[i].cbegin(), c2[i].cend());
            c2.erase(c2.begin() + i);
            found_overlap = true;
            changed = true;
            break;

          } else{
            i++;
          }
        }

        if (!found_overlap){
          c2.push_back(c1.back());
          c1.pop_back();
          changed = true;
        }
      }

      return changed;
    }
  };

}