#include "Graph.h"
#include "Layout.h"
#include <cassert>
#include <iostream>

struct TestNodeData {
  int type_id;
};
struct TestEdgeData {
  float weight;
};
struct TestPortData {
  int pin_dir;
};

void test_graph_creation() {
  Graph<TestNodeData, TestEdgeData, TestPortData> g;

  int n1 = g.addNode({0, 0, 10, 10}, {1});
  int n2 = g.addNode({20, 20, 10, 10}, {2});

  assert(g.getNodes().size() == 2);

  int p1 = g.addPort(n1, PortType::Output, {10, 5}, {0});
  int p2 = g.addPort(n2, PortType::Input, {0, 5}, {1});

  assert(g.getNode(n1).ports.size() == 1);
  assert(g.getNode(n2).ports.size() == 1);

  int e1 = g.addEdge(n1, p1, n2, p2, {1.5f});

  assert(g.getEdges().size() == 1);
  assert(g.getNode(n1).ports[0].connected_edge_ids[0] == e1);

  std::cout << "Graph creation test passed.\n";
}

void test_quadtree_culling() {
  Graph<TestNodeData, TestEdgeData, TestPortData> g;

  for (int i = 0; i < 100; i++) {
    g.addNode({static_cast<float>(i * 10), 0, 5, 5}, {1});
  }

  // Query a small area that should only contain 2 nodes (x=0 to x=15)
  Rect view = {0, -10, 15, 20};
  auto visible = g.queryVisibleNodes(view);

  assert(visible.size() == 2);
  assert(visible[0] == 0);
  assert(visible[1] == 1);

  std::cout << "Quadtree culling test passed.\n";
}

void test_manhattan_layout() {
  Graph<TestNodeData, TestEdgeData, TestPortData> g;

  int n1 = g.addNode({0, 0, 10, 10}, {1});
  int n2 = g.addNode({0, 0, 10, 10}, {2}); // Same initial pos

  int p1 = g.addPort(n1, PortType::Output, {10, 5}, {0});
  int p2 = g.addPort(n2, PortType::Input, {0, 5}, {1});

  g.addEdge(n1, p1, n2, p2, {1.0f});

  Layout::applyManhattanLayout(g);

  // Check nodes are separated
  assert(g.getNode(n1).bounds.x != g.getNode(n2).bounds.x ||
         g.getNode(n1).bounds.y != g.getNode(n2).bounds.y);

  // Check edge has waypoints
  auto &edge = g.getEdges()[0];
  assert(edge.waypoints.size() == 4); // Start, Cut1, Cut2, End

  // Check orthogonal segments
  for (size_t i = 0; i < edge.waypoints.size() - 1; i++) {
    Vec2 p1 = edge.waypoints[i];
    Vec2 p2 = edge.waypoints[i + 1];

    bool isHorizontal = (p1.y == p2.y);
    bool isVertical = (p1.x == p2.x);
    assert(isHorizontal || isVertical); // Segments must be orthogonal
  }

  std::cout << "Manhattan layout test passed.\n";
}

int main() {
  std::cout << "Running Graph Unit Tests...\n";
  test_graph_creation();
  test_quadtree_culling();
  test_manhattan_layout();
  std::cout << "All tests passed!\n";
  return 0;
}
