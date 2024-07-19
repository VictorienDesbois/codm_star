#include <gtest/gtest.h>

#include "codm_types.hpp"

using namespace codm_star;


MetaAgents vec_to_ma(
  std::vector<std::vector<Agent>> v
) {
  MetaAgents result;
  
  for (auto ma: v) {
    MetaAgent result_ma;
    result_ma.insert(ma.begin(), ma.end());
    result.push_back(result_ma);
  }

  return result;
}


bool is_same_mas(MetaAgents mas1, MetaAgents mas2) {
  if (mas1.size() != mas2.size()) {
    return false;
  }

  size_t ma_len = mas1.size();
  size_t count = 0;

  for (MetaAgent ma1: mas1) {
    for (MetaAgent ma2: mas2) {
      if (ma1 == ma2) {
        count++;
        break;
      }
    }
  }

  return count == ma_len;
}


TEST(cleaning, positive) {
  MetaAgentManager ma_manager;

  MetaAgents ma1 = vec_to_ma({{1,2,3},{3},{4},{5,6},{5,6,7}});
  MetaAgents ma2 = vec_to_ma({{4},{0,1,2}});

  ma_manager.meta_agents_set_union(ma1, ma2);
  MetaAgents true_result = vec_to_ma({{0,1,2,3},{4},{5,6,7}});

  EXPECT_EQ((is_same_mas(ma2, true_result)), true);
}


TEST(cleaning_two, positive) {
  MetaAgentManager ma_manager;

  MetaAgents ma1 = vec_to_ma({{1},{3},{4},{5},{6}});
  MetaAgents ma2 = vec_to_ma({{1,6}});

  ma_manager.meta_agents_set_union(ma1, ma2);
  MetaAgents true_result = vec_to_ma({{3},{4},{5},{1,6}});

  EXPECT_EQ((is_same_mas(ma2, true_result)), true);
}


TEST(cleaning_three, positive) {
  MetaAgentManager ma_manager;

  MetaAgents ma1 = vec_to_ma({{0,5},{1},{2},{3},{4},{6,10},{7},{8,9}});
  MetaAgents ma2 = vec_to_ma({{1,5}});

  ma_manager.meta_agents_set_union(ma1, ma2);
  MetaAgents true_result = vec_to_ma({{0,1,5},{2},{3},{4},{6,10},{7},{8,9}});

  EXPECT_EQ((is_same_mas(ma2, true_result)), true);
}


TEST(cleaning_four, positive) {
  MetaAgentManager ma_manager;

  MetaAgents ma1 = vec_to_ma({{0,10},{1,7},{2,4},{3,6},{5},{8},{9},{11},{12},{13},{14}});
  MetaAgents ma2 = vec_to_ma({{4,6}});

  ma_manager.meta_agents_set_union(ma1, ma2);
  MetaAgents true_result = vec_to_ma({{0,10},{1,7},{2,3,4,6},{5},{8},{9},{11},{12},{13},{14}});

  EXPECT_EQ((is_same_mas(ma2, true_result)), true);
}


TEST(cleaning_five, positive) {
  MetaAgentManager ma_manager;

  MetaAgents ma1 = vec_to_ma({{0,1},{1,2,3},{3,2,1}});
  MetaAgents ma2 = vec_to_ma({{4,5,6,7}});

  ma_manager.meta_agents_set_union(ma1, ma2);
  MetaAgents true_result = vec_to_ma({{0,1,2,3},{4,5,6,7}});

  for (MetaAgent ma: ma2) {
    std::string debug_str = "";
    for (Agent a: ma) {
      debug_str += std::to_string(a) + ",";
    }
    LOG_DEBUG(debug_str);
  }

  EXPECT_EQ((is_same_mas(ma2, true_result)), true);
}