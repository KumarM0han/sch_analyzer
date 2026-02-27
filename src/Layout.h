#pragma once
#include "Graph.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/energybased/FMMMLayout.h>
#include <ogdf/layered/MedianHeuristic.h>
#include <ogdf/layered/OptimalHierarchyLayout.h>
#include <ogdf/layered/OptimalRanking.h>
#include <ogdf/layered/SugiyamaLayout.h>
#include <ogdf/planarity/PlanarizationLayout.h>


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
void applyOGDFLayout(Graph<NodeData, EdgeData, PortData> &customGraph) {
  auto &nodes = customGraph.getNodes();
  if (nodes.empty())
    return;

  // Initialize OGDF Graph and Attributes
  ogdf::Graph G;
  ogdf::GraphAttributes GA(G, ogdf::GraphAttributes::nodeGraphics |
                                  ogdf::GraphAttributes::edgeGraphics);

  // Map from our custom graph node ID to OGDF Node
  std::unordered_map<int, ogdf::node> idToNode;

  for (size_t i = 0; i < nodes.size(); ++i) {
    if (!nodes[i].visible)
      continue;
    ogdf::node n = G.newNode();
    idToNode[nodes[i].id] = n;

    // Set initial size with swapped W and H for Left-To-Right orientation
    GA.width(n) = nodes[i].bounds.h;
    GA.height(n) = nodes[i].bounds.w;
  }

  // Create Edges
  auto &edges = customGraph.getEdges();
  std::unordered_map<int, ogdf::edge> idToEdge;
  for (size_t i = 0; i < edges.size(); ++i) {
    auto src_it = idToNode.find(edges[i].src_node_id);
    auto dst_it = idToNode.find(edges[i].dst_node_id);
    if (src_it != idToNode.end() && dst_it != idToNode.end()) {
      ogdf::edge e = G.newEdge(src_it->second, dst_it->second);
      idToEdge[edges[i].id] = e;
    }
  }

  if (customGraph.current_layout == LayoutAlgorithm::Sugiyama) {
    // Set up Sugiyama Layout
    ogdf::SugiyamaLayout SL;
    SL.setRanking(new ogdf::OptimalRanking);
    SL.setCrossMin(new ogdf::MedianHeuristic);

    ogdf::OptimalHierarchyLayout *ohl = new ogdf::OptimalHierarchyLayout;
    ohl->layerDistance(100.0);
    ohl->nodeDistance(50.0);

    SL.setLayout(ohl);
    SL.call(GA);
  } else if (customGraph.current_layout == LayoutAlgorithm::FMMM) {
    ogdf::FMMMLayout fmmm;
    fmmm.useHighLevelOptions(true);
    fmmm.unitEdgeLength(50.0);
    fmmm.newInitialPlacement(true);
    fmmm.qualityVersusSpeed(
        ogdf::FMMMOptions::QualityVsSpeed::GorgeousAndEfficient);
    fmmm.call(GA);
  } else {
    // Set up Orthogonal Layout via Planarization
    ogdf::PlanarizationLayout PL;
    PL.call(GA);
  }

  // Sync back to custom structures (swapping X and Y back to get LTR)
  for (size_t i = 0; i < nodes.size(); ++i) {
    auto it = idToNode.find(nodes[i].id);
    if (it != idToNode.end()) {
      nodes[i].bounds.x = GA.y(it->second) - nodes[i].bounds.w / 2.0;
      nodes[i].bounds.y = GA.y(it->second) - nodes[i].bounds.h / 2.0;
      // Note: OGDF coordinate output has x and y. Because we swapped w and h,
      // the layout ran Top-to-Bottom. GA.x() is the rank (horizontal position
      // in TTB), GA.y() is the horizontal layers. Wait, top-to-bottom means y
      // is the layers. For LTR, x should be the layers. So customGraph X =
      // GA.y(), customGraph Y = GA.x()
      nodes[i].bounds.x = GA.y(it->second) - nodes[i].bounds.w / 2.0;
      nodes[i].bounds.y = GA.x(it->second) - nodes[i].bounds.h / 2.0;
    }
  }

  for (auto &edge : edges) {
    auto it = idToEdge.find(edge.id);
    if (it != idToEdge.end()) {
      edge.waypoints.clear();
      ogdf::DPolyline &poly = GA.bends(it->second);

      // Port positions for start and end
      auto &src_node = customGraph.getNode(edge.src_node_id);
      auto &dst_node = customGraph.getNode(edge.dst_node_id);
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

      edge.waypoints.push_back(src_pos);
      for (ogdf::DPoint p : poly) {
        edge.waypoints.push_back({(float)p.m_y, (float)p.m_x}); // Swap Y and X
      }
      edge.waypoints.push_back(dst_pos);
    }
  }

  // Make sure quadtree is updated after layout moves
  customGraph.rebuildQuadtree();
}

} // namespace Layout
