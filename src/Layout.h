#pragma once
#include "Graph.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace Layout {

// Helper to determine if two lines intersect
inline bool linesIntersect(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4) {
  auto ccw = [](Vec2 A, Vec2 B, Vec2 C) {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
  };
  return ccw(p1, p3, p4) != ccw(p2, p3, p4) &&
         ccw(p1, p2, p3) != ccw(p1, p2, p4);
}

template <typename NodeData, typename EdgeData, typename PortData>
void applyManhattanLayout(Graph<NodeData, EdgeData, PortData> &graph) {
  auto &nodes = graph.getNodes();
  if (nodes.empty())
    return;

  // Simple grid placement for overlap prevention
  int col_count = std::ceil(std::sqrt(nodes.size()));
  float grid_spacing_X = 200.0f;
  float grid_spacing_Y = 150.0f;

  for (size_t i = 0; i < nodes.size(); ++i) {
    int row = i / col_count;
    int col = i % col_count;

    nodes[i].bounds.x = col * grid_spacing_X;
    nodes[i].bounds.y = row * grid_spacing_Y;
  }

  // Make sure quadtree is updated after layout moves
  graph.reresolveQuadtree(col_count * grid_spacing_X,
                          ((nodes.size() / col_count) + 1) * grid_spacing_Y);

  // Calculate Orthogonal Routes
  auto &edges = graph.getEdges();
  for (auto &edge : edges) {
    edge.waypoints.clear();

    auto &src_node = graph.getNode(edge.src_node_id);
    auto &dst_node = graph.getNode(edge.dst_node_id);

    Vec2 src_pos = {0, 0};
    Vec2 dst_pos = {0, 0};

    for (const auto &p : src_node.ports) {
      if (p.id == edge.src_port_id) {
        src_pos = {src_node.bounds.x + p.local_pos.x,
                   src_node.bounds.y + p.local_pos.y};
        break;
      }
    }
    for (const auto &p : dst_node.ports) {
      if (p.id == edge.dst_port_id) {
        dst_pos = {dst_node.bounds.x + p.local_pos.x,
                   dst_node.bounds.y + p.local_pos.y};
        break;
      }
    }

    // Start point
    edge.waypoints.push_back(src_pos);

    // Left-to-right enforcement implies outgoing on right, incoming on left
    // Standard Manhattan 3-segment route: Move out horizontally, move
    // vertically, move in horizontally

    float mid_x = (src_pos.x + dst_pos.x) / 2.0f;

    // Cutpoint 1: Move right from source
    edge.waypoints.push_back({mid_x, src_pos.y});

    // Cutpoint 2: Move up/down to match dest Y
    edge.waypoints.push_back({mid_x, dst_pos.y});

    // End point (moves right into dest)
    edge.waypoints.push_back(dst_pos);
  }
}
} // namespace Layout
