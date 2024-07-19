#include <gtest/gtest.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "graph.hpp"


enum { node_0, node_1, node_2, node_3, node_4 };

CommunicationsGraph get_graph_one() {
/**
 * graph
 *     0---1
 *     |   |
 * 4---3   2
 */

  CommunicationsGraph g = CommunicationsGraph();
  
  g.add_edge(node_0, node_1);
  g.add_edge(node_0, node_3);
  g.add_edge(node_1, node_2);
  g.add_edge(node_3, node_4);

  return g;
}

MovesGraph get_graph_two() {
/**
 * Movement graph
 *       0<--->1
 *       ^     ^
 *       |     |
 *       v     v
 * 4<--->3<--->2
 */

  MovesGraph g = MovesGraph();

  return g;
}

TEST(is_graph_connected, positive) {
	CommunicationsGraph g_com = get_graph_one();
  
  std::vector<AgentPosition> config = {node_0, node_1};
  Configuration c;
  c.insert(c.begin(), config.begin(), config.end());
	EXPECT_EQ(g_com.is_configuration_connected(c), true);
}

TEST(is_graph_connected, negative) {
  CommunicationsGraph g_com = get_graph_one();

  std::vector<AgentPosition> config = {node_3, node_2};
  Configuration c;
  c.insert(c.begin(), config.begin(), config.end());
  EXPECT_EQ(g_com.is_configuration_connected(c), false);
}

TEST(is_graph_connected_two, negative) {
  CommunicationsGraph g_com = get_graph_one();

  std::vector<AgentPosition> config = {node_1, node_2, node_3};
  Configuration c;
  c.insert(c.begin(), config.begin(), config.end());
  EXPECT_EQ(g_com.is_configuration_connected(c), false);
}

TEST(is_graph_connected_two, positive) {
  CommunicationsGraph g_com = get_graph_one();
  // this configuration should be connected (: all the graph)
  std::vector<AgentPosition> config = {node_0, node_1, node_2, node_3, node_4};
  Configuration c;
  c.insert(c.begin(), config.begin(), config.end());
  EXPECT_EQ(g_com.is_configuration_connected(c), true);
}

TEST(is_graph_connected_three, negative) {
  CommunicationsGraph g_com = get_graph_one();
  std::vector<AgentPosition> config = {node_1, node_2, node_3, node_4};
  Configuration c;
  c.insert(c.begin(), config.begin(), config.end());
  EXPECT_EQ(g_com.is_configuration_connected(c), false);
}

