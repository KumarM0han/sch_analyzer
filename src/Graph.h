#pragma once

#include <memory>
#include <string>
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
    }
  }

  void retrieve(std::vector<int> &returnObjects, const Rect &pRect) const {
    int index = getIndex(pRect);
    if (index != -1 && nodes[0]) {
      nodes[index]->retrieve(returnObjects, pRect);
    } else if (nodes[0]) {
      // Need to check all children if it overlaps multiple quadrants
      for (int i = 0; i < 4; i++) {
        if (pRect.intersects(nodes[i]->bounds)) {
          nodes[i]->retrieve(returnObjects, pRect);
        }
      }
    }

    for (int id : node_ids) {
      returnObjects.push_back(id);
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

  Graph() {
    quadtree = std::make_unique<Quadtree<NodeData, EdgeData, PortData>>(
        0, Rect{-100000.0f, -100000.0f, 200000.0f, 200000.0f}, *this);
  }

  void reresolveQuadtree(float width, float height) {
    quadtree = std::make_unique<Quadtree<NodeData, EdgeData, PortData>>(
        0, Rect{-width / 2, -height / 2, width, height}, *this);
    for (const auto &node : nodes) {
      quadtree->insert(node.id, node.bounds);
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
    std::vector<int> visible_ids;
    quadtree->retrieve(visible_ids, viewBounds);

    // Filter out actual intersections since Quadtree retrieve is broad
    std::vector<int> exact_ids;
    for (int id : visible_ids) {
      if (nodes[id].bounds.intersects(viewBounds)) {
        if (!global_hide_all || nodes[id].visible) {
          exact_ids.push_back(id);
        }
      }
    }
    return exact_ids;
  }
};
