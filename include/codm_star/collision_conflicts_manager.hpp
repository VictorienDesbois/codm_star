#pragma once

#include "cmapf_types.hpp"
#include "graph.hpp"


namespace codm_star {

  struct CollisionsManager {

    /**
     * @brief Get the groups of agents that are in collision.
     * 
     * @param s The first Configuration to analyze.
     * @param t The second Configuration to analyze.
     * @return Returns the groups of agents that are in collisions.
     */
    MetaAgents get_coupled_agent(
      const Configuration &s, 
      const Configuration &t
    ) {
      assert(s.size() == t.size());
      size_t nb_agents_ = s.size();

      MetaAgents meta_agents_set;

      for (Agent a = 0; a < nb_agents_; a++) if (t[a] >= 0) {
        for (Agent b = a + 1; b < nb_agents_; b++) if (t[b] >= 0) {

          if (t[a] == t[b]) {
            std::vector<Agent> agent_couple = {a,b};
            
            MetaAgent ma;
            ma.insert(agent_couple.begin(), agent_couple.end());
            meta_agents_set.push_back(ma);
          }

        }
      }

      return meta_agents_set;
    }


    /**
     * @brief Get the groups of agents that are in collision.
     * Manage the swapp collision constraint.
     * 
     * todo: merge those two functions
     * 
     * @param s The first Configuration to analyze.
     * @param t The second Configuration to analyze.
     * @return Returns the groups of agents that are in collisions.
     */
    MetaAgents get_coupled_agent_swap_collision(
      const Configuration &s, 
      const Configuration &t
    ) {
      assert(s.size() == t.size());
      size_t nb_agents_ = s.size();

      MetaAgents meta_agents_set;

      for (Agent a = 0; a < nb_agents_; a++) if (t[a] >= 0) {
        for (Agent b = a + 1; b < nb_agents_; b++) if (t[b] >= 0) {

          bool vertex_collision   = (t[a] == t[b]);
          bool swapping_collision = (s[a] == t[b]) && (s[b] == t[a]);

          if (vertex_collision || swapping_collision) {
            std::vector<Agent> agent_couple = {a,b};
            
            MetaAgent ma;
            ma.insert(agent_couple.begin(), agent_couple.end());
            meta_agents_set.push_back(ma);
          }
        }
      }

      return meta_agents_set;
    }

  };

}