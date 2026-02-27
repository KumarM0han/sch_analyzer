#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declarations
template <typename NodeData, typename EdgeData, typename PortData> class Graph;
template <typename NodeData, typename EdgeData, typename PortData> class Node;
template <typename NodeData, typename EdgeData, typename PortData> class Edge;

struct Vec2 {
  float x;
  float y;

  bool operator==(const Vec2 &other) const {
    return x == other.x && y == other.y;
  }
};

struct Rect {
  float x;
  float y;
  float w;
  float h;

  bool contains(const Vec2 &p) const {
    return (p.x >= x && p.x <= x + w && p.y >= y && p.y <= y + h);
  }

  bool intersects(const Rect &other) const {
    return !(other.x > x + w || other.x + other.w < x || other.y > y + h ||
             other.y + other.h < y);
  }
};

enum class PortType {
  Input, // Left side
  Output // Right side
};

enum class LayoutAlgorithm { Sugiyama, FastHierarchy, FMMM };

struct LayoutState {
  bool is_cached = false;
  std::vector<Rect> node_bounds;
  std::vector<std::vector<Vec2>> edge_waypoints;
};

template <typename NodeData, typename EdgeData, typename PortData> class Port {
public:
  int id;
  int node_id;
  PortType type;
  Vec2 local_pos; // Relative to node's top-left
  PortData data;
  bool visible;
  std::vector<int> connected_edge_ids;

  Port(int id, int node_id, PortType type, Vec2 local_pos, const PortData &data)
      : id(id), node_id(node_id), type(type), local_pos(local_pos), data(data),
        visible(true) {}
};

template <typename NodeData, typename EdgeData, typename PortData> class Edge {
public:
  int id;
  int src_node_id;
  int src_port_id;
  int dst_node_id;
  int dst_port_id;

  std::vector<Vec2> waypoints; // Includes cutpoints for orthogonal routing
  EdgeData data;
  bool visible;

  Edge(int id, int src_node, int src_port, int dst_node, int dst_port,
       const EdgeData &data)
      : id(id), src_node_id(src_node), src_port_id(src_port),
        dst_node_id(dst_node), dst_port_id(dst_port), data(data),
        visible(false) {}
};

template <typename NodeData, typename EdgeData, typename PortData> class Node {
public:
  int id;
  Rect bounds;
  NodeData data;
  bool visible;
  std::vector<Port<NodeData, EdgeData, PortData>> ports;

  Node(int id, Rect bounds, const NodeData &data)
      : id(id), bounds(bounds), data(data), visible(true) {}

  void addPort(const Port<NodeData, EdgeData, PortData> &port) {
    ports.push_back(port);
  }
};

// --- Quadtree Implementation ---
template <typename NodeData, typename EdgeData, typename PortData>
class Quadtree {
private:
  static const int MAX_OBJECTS = 10;
  static const int MAX_LEVELS = 5;

  int level;
  Rect bounds;
  std::vector<int> node_ids; // Store Node IDs instead of pointers
  std::vector<int> edge_ids; // Store Edge IDs for culling
  std::unique_ptr<Quadtree> nodes[4];
  Graph<NodeData, EdgeData, PortData> &graph;

  void split() {
    float subWidth = bounds.w / 2.0f;
    float subHeight = bounds.h / 2.0f;
    float x = bounds.x;
    float y = bounds.y;

    nodes[0] = std::make_unique<Quadtree>(
        level + 1, Rect{x + subWidth, y, subWidth, subHeight}, graph);
    nodes[1] = std::make_unique<Quadtree>(
        level + 1, Rect{x, y, subWidth, subHeight}, graph);
    nodes[2] = std::make_unique<Quadtree>(
        level + 1, Rect{x, y + subHeight, subWidth, subHeight}, graph);
    nodes[3] = std::make_unique<Quadtree>(
        level + 1, Rect{x + subWidth, y + subHeight, subWidth, subHeight},
        graph);
  }

  int getIndex(const Rect &pRect) const {
    int index = -1;
    float verticalMidpoint = bounds.x + (bounds.w / 2.0f);
    float horizontalMidpoint = bounds.y + (bounds.h / 2.0f);

    bool topQuadrant = (pRect.y < horizontalMidpoint &&
                        pRect.y + pRect.h < horizontalMidpoint);
    bool bottomQuadrant = (pRect.y > horizontalMidpoint);

    if (pRect.x < verticalMidpoint && pRect.x + pRect.w < verticalMidpoint) {
      if (topQuadrant)
        index = 1;
      else if (bottomQuadrant)
        index = 2;
    } else if (pRect.x > verticalMidpoint) {
      if (topQuadrant)
        index = 0;
      else if (bottomQuadrant)
        index = 3;
    }
    return index;
  }

public:
  Quadtree(int pLevel, Rect pBounds,
           Graph<NodeData, EdgeData, PortData> &pGraph)
      : level(pLevel), bounds(pBounds), graph(pGraph) {}

  void clear() {
    node_ids.clear();
    edge_ids.clear();
    for (int i = 0; i < 4; i++) {
      if (nodes[i]) {
        nodes[i]->clear();
        nodes[i] = nullptr;
      }
    }
  }

  void insert(int node_id, const Rect &pRect) {
    if (nodes[0]) {
      int index = getIndex(pRect);
      if (index != -1) {
        nodes[index]->insert(node_id, pRect);
        return;
      }
    }

    node_ids.push_back(node_id);

    if (node_ids.size() > MAX_OBJECTS && level < MAX_LEVELS) {
      if (!nodes[0])
        split();

      int i = 0;
      while (i < node_ids.size()) {
        const auto &nodeBounds = graph.getNode(node_ids[i]).bounds;
        int index = getIndex(nodeBounds);
        if (index != -1) {
          nodes[index]->insert(node_ids[i], nodeBounds);
          node_ids.erase(node_ids.begin() + i);
        } else {
          i++;
        }
      }

      i = 0;
      // also re-distribute edge_ids
      while (i < edge_ids.size()) {
        const auto &edge = graph.getEdge(edge_ids[i]);
        Rect edgeBounds = {edge.waypoints[0].x, edge.waypoints[0].y, 0, 0};
        for (const auto &wp : edge.waypoints) {
          if (wp.x < edgeBounds.x) {
            edgeBounds.w += edgeBounds.x - wp.x;
            edgeBounds.x = wp.x;
          } else if (wp.x > edgeBounds.x + edgeBounds.w)
            edgeBounds.w = wp.x - edgeBounds.x;
          if (wp.y < edgeBounds.y) {
            edgeBounds.h += edgeBounds.y - wp.y;
            edgeBounds.y = wp.y;
          } else if (wp.y > edgeBounds.y + edgeBounds.h)
            edgeBounds.h = wp.y - edgeBounds.y;
        }
        int index = getIndex(edgeBounds);
        if (index != -1) {
          nodes[index]->insertEdge(edge_ids[i], edgeBounds);
          edge_ids.erase(edge_ids.begin() + i);
        } else {
          i++;
        }
      }
    }
  }

  void insertEdge(int edge_id, const Rect &pRect) {
    if (nodes[0]) {
      int index = getIndex(pRect);
      if (index != -1) {
        nodes[index]->insertEdge(edge_id, pRect);
        return;
      }
    }
    edge_ids.push_back(edge_id);
    // Max capacity triggers split but we'll re-distribute nodes and edges in
    // insert just piggy back onto node limits for simplicity since edges follow
    // nodes closely
  }

  void retrieve(std::vector<int> &returnNodes, std::vector<int> &returnEdges,
                const Rect &pRect) const {
    int index = getIndex(pRect);
    if (index != -1 && nodes[0]) {
      nodes[index]->retrieve(returnNodes, returnEdges, pRect);
    } else if (nodes[0]) {
      // Need to check all children if it overlaps multiple quadrants
      for (int i = 0; i < 4; i++) {
        if (pRect.intersects(nodes[i]->bounds)) {
          nodes[i]->retrieve(returnNodes, returnEdges, pRect);
        }
      }
    }

    for (int id : node_ids) {
      returnNodes.push_back(id);
    }
    for (int id : edge_ids) {
      returnEdges.push_back(id);
    }
  }
};

template <typename NodeData, typename EdgeData, typename PortData> class Graph {
private:
  std::vector<Node<NodeData, EdgeData, PortData>> nodes;
  std::vector<Edge<NodeData, EdgeData, PortData>> edges;
  std::unique_ptr<Quadtree<NodeData, EdgeData, PortData>> quadtree;
  int next_node_id = 0;
  int next_edge_id = 0;
  int next_port_id = 0;

public:
  bool global_hide_all = false;
  LayoutAlgorithm current_layout = LayoutAlgorithm::FastHierarchy;
  std::unordered_map<LayoutAlgorithm, LayoutState> layout_cache;

  Graph() {
    quadtree = std::make_unique<Quadtree<NodeData, EdgeData, PortData>>(
        0, Rect{-100000.0f, -100000.0f, 200000.0f, 200000.0f}, *this);
  }

  void rebuildQuadtree() {
    if (nodes.empty())
      return;

    float min_x = nodes[0].bounds.x;
    float min_y = nodes[0].bounds.y;
    float max_x = nodes[0].bounds.x + nodes[0].bounds.w;
    float max_y = nodes[0].bounds.y + nodes[0].bounds.h;

    for (size_t i = 1; i < nodes.size(); ++i) {
      const auto &node = nodes[i];
      if (node.bounds.x < min_x)
        min_x = node.bounds.x;
      if (node.bounds.y < min_y)
        min_y = node.bounds.y;
      if (node.bounds.x + node.bounds.w > max_x)
        max_x = node.bounds.x + node.bounds.w;
      if (node.bounds.y + node.bounds.h > max_y)
        max_y = node.bounds.y + node.bounds.h;
    }

    // Add padding to bounds
    min_x -= 1000.0f;
    min_y -= 1000.0f;
    max_x += 1000.0f;
    max_y += 1000.0f;

    quadtree = std::make_unique<Quadtree<NodeData, EdgeData, PortData>>(
        0, Rect{min_x, min_y, max_x - min_x, max_y - min_y}, *this);

    for (const auto &node : nodes) {
      quadtree->insert(node.id, node.bounds);
    }
    for (const auto &edge : edges) {
      if (edge.waypoints.empty())
        continue;
      Rect edgeBounds = {edge.waypoints[0].x, edge.waypoints[0].y, 0, 0};
      for (const auto &wp : edge.waypoints) {
        if (wp.x < edgeBounds.x) {
          edgeBounds.w += edgeBounds.x - wp.x;
          edgeBounds.x = wp.x;
        } else if (wp.x > edgeBounds.x + edgeBounds.w)
          edgeBounds.w = wp.x - edgeBounds.x;
        if (wp.y < edgeBounds.y) {
          edgeBounds.h += edgeBounds.y - wp.y;
          edgeBounds.y = wp.y;
        } else if (wp.y > edgeBounds.y + edgeBounds.h)
          edgeBounds.h = wp.y - edgeBounds.y;
      }
      quadtree->insertEdge(edge.id, edgeBounds);
    }
  }

  int addNode(Rect bounds, const NodeData &data) {
    int id = next_node_id++;
    nodes.emplace_back(id, bounds, data);
    quadtree->insert(id, bounds);
    return id;
  }

  int addPort(int node_id, PortType type, Vec2 local_pos,
              const PortData &data) {
    if (node_id < 0 || node_id >= nodes.size())
      return -1;
    int id = next_port_id++;
    Port<NodeData, EdgeData, PortData> port(id, node_id, type, local_pos, data);
    nodes[node_id].addPort(port);
    return id;
  }

  int addEdge(int src_node, int src_port, int dst_node, int dst_port,
              const EdgeData &data) {
    int id = next_edge_id++;
    edges.emplace_back(id, src_node, src_port, dst_node, dst_port, data);

    // Connect port references
    for (auto &port : nodes[src_node].ports) {
      if (port.id == src_port)
        port.connected_edge_ids.push_back(id);
    }
    for (auto &port : nodes[dst_node].ports) {
      if (port.id == dst_port)
        port.connected_edge_ids.push_back(id);
    }
    return id;
  }

  Node<NodeData, EdgeData, PortData> &getNode(int id) { return nodes[id]; }
  const Node<NodeData, EdgeData, PortData> &getNode(int id) const {
    return nodes[id];
  }

  Edge<NodeData, EdgeData, PortData> &getEdge(int id) { return edges[id]; }
  const Edge<NodeData, EdgeData, PortData> &getEdge(int id) const {
    return edges[id];
  }

  std::vector<Node<NodeData, EdgeData, PortData>> &getNodes() { return nodes; }
  std::vector<Edge<NodeData, EdgeData, PortData>> &getEdges() { return edges; }

  std::vector<int> queryVisibleNodes(const Rect &viewBounds) {
    std::vector<int> visible_nodes;
    std::vector<int> visible_edges;
    quadtree->retrieve(visible_nodes, visible_edges, viewBounds);

    // Filter out actual intersections since Quadtree retrieve is broad
    std::vector<int> exact_ids;
    for (int id : visible_nodes) {
      if (nodes[id].bounds.intersects(viewBounds)) {
        if (!global_hide_all || nodes[id].visible) {
          exact_ids.push_back(id);
        }
      }
    }
    return exact_ids;
  }

  std::vector<int> queryVisibleEdges(const Rect &viewBounds) {
    std::vector<int> visible_nodes;
    std::vector<int> visible_edges;
    quadtree->retrieve(visible_nodes, visible_edges, viewBounds);
    return visible_edges;
  }

  void saveLayoutToCache(LayoutAlgorithm algo) {
    LayoutState state;
    state.is_cached = true;
    state.node_bounds.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
      state.node_bounds[i] = nodes[i].bounds;
    }
    state.edge_waypoints.resize(edges.size());
    for (size_t i = 0; i < edges.size(); ++i) {
      state.edge_waypoints[i] = edges[i].waypoints;
    }
    layout_cache[algo] = state;
  }

  bool loadLayoutFromCache(LayoutAlgorithm algo) {
    if (layout_cache.find(algo) != layout_cache.end() &&
        layout_cache[algo].is_cached) {
      const auto &state = layout_cache[algo];
      for (size_t i = 0; i < nodes.size(); ++i) {
        nodes[i].bounds = state.node_bounds[i];
      }
      for (size_t i = 0; i < edges.size(); ++i) {
        edges[i].waypoints = state.edge_waypoints[i];
      }
      rebuildQuadtree();
      current_layout = algo;
      return true;
    }
    return false;
  }
};
