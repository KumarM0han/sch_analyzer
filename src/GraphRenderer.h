#pragma once

#include "Graph.h"
#include "Layout.h"
#include "imgui.h"
#include <cmath>
#include <string>
#include <vector>

struct PanZoomState {
  Vec2 pan = {0.0f, 0.0f};
  float zoom = 1.0f;

  Vec2 screenToWorld(const Vec2 &screenPos) const {
    return {(screenPos.x - pan.x) / zoom, (screenPos.y - pan.y) / zoom};
  }

  Vec2 worldToScreen(const Vec2 &worldPos) const {
    return {worldPos.x * zoom + pan.x, worldPos.y * zoom + pan.y};
  }

  Rect worldToScreenRect(const Rect &worldRect) const {
    return {worldRect.x * zoom + pan.x, worldRect.y * zoom + pan.y,
            worldRect.w * zoom, worldRect.h * zoom};
  }
};

enum class SelectionType { None, Node, Edge, Port };

struct SelectionState {
  SelectionType type = SelectionType::None;
  int id = -1;
  void *data = nullptr;

  void clear() {
    type = SelectionType::None;
    id = -1;
    data = nullptr;
  }
};

inline float dist_to_segment_squared(Vec2 v, Vec2 w, Vec2 p) {
  float l2 = (w.x - v.x) * (w.x - v.x) + (w.y - v.y) * (w.y - v.y);
  if (l2 == 0.0f)
    return (p.x - v.x) * (p.x - v.x) + (p.y - v.y) * (p.y - v.y);
  float t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;
  t = std::max(0.0f, std::min(1.0f, t));
  Vec2 proj = {v.x + t * (w.x - v.x), v.y + t * (w.y - v.y)};
  return (p.x - proj.x) * (p.x - proj.x) + (p.y - proj.y) * (p.y - proj.y);
}

template <typename NodeData, typename EdgeData, typename PortData>
class GraphRenderer {
private:
  Graph<NodeData, EdgeData, PortData> &graph;
  PanZoomState camera;
  SelectionState selection;

  // Config colors
  ImU32 col_bg = IM_COL32(30, 30, 30, 255);
  ImU32 col_grid = IM_COL32(200, 200, 200, 40);
  ImU32 col_node = IM_COL32(70, 75, 80, 255);
  ImU32 col_node_selected = IM_COL32(200, 150, 50, 255);
  ImU32 col_node_border = IM_COL32(180, 180, 180, 255);
  ImU32 col_port = IM_COL32(150, 200, 150, 255);
  ImU32 col_edge = IM_COL32(170, 170, 170, 255);
  ImU32 col_edge_selected = IM_COL32(255, 100, 50, 255);

  bool isDraggingEmpty = false;

  void drawGrid(ImDrawList *draw_list, ImVec2 canvas_pos, ImVec2 canvas_size) {
    float GRID_STEP = 64.0f * camera.zoom;
    for (float x = fmodf(camera.pan.x, GRID_STEP); x < canvas_size.x;
         x += GRID_STEP)
      draw_list->AddLine(ImVec2(canvas_pos.x + x, canvas_pos.y),
                         ImVec2(canvas_pos.x + x, canvas_pos.y + canvas_size.y),
                         col_grid);
    for (float y = fmodf(camera.pan.y, GRID_STEP); y < canvas_size.y;
         y += GRID_STEP)
      draw_list->AddLine(ImVec2(canvas_pos.x, canvas_pos.y + y),
                         ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + y),
                         col_grid);
  }

public:
  GraphRenderer(Graph<NodeData, EdgeData, PortData> &g) : graph(g) {}

  void render() {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("View")) {
        if (ImGui::MenuItem("Reset View")) {
          camera.pan = {0.0f, 0.0f};
          camera.zoom = 1.0f;
        }
        ImGui::Separator();
        // Layout algorithm toggle
        bool isOrtho = (graph.current_layout == LayoutAlgorithm::Orthogonal);
        if (ImGui::MenuItem("Orthogonal Layout", nullptr, isOrtho)) {
          if (!graph.loadLayoutFromCache(LayoutAlgorithm::Orthogonal)) {
            graph.current_layout = LayoutAlgorithm::Orthogonal;
            Layout::applyOGDFLayout(graph);
            graph.saveLayoutToCache(LayoutAlgorithm::Orthogonal);
          }
        }
        bool isSugi = (graph.current_layout == LayoutAlgorithm::Sugiyama);
        if (ImGui::MenuItem("Sugiyama Layout", nullptr, isSugi)) {
          if (!graph.loadLayoutFromCache(LayoutAlgorithm::Sugiyama)) {
            graph.current_layout = LayoutAlgorithm::Sugiyama;
            Layout::applyOGDFLayout(graph);
            graph.saveLayoutToCache(LayoutAlgorithm::Sugiyama);
          }
        }
        bool isFMMM = (graph.current_layout == LayoutAlgorithm::FMMM);
        if (ImGui::MenuItem("FMMM Layout", nullptr, isFMMM)) {
          if (!graph.loadLayoutFromCache(LayoutAlgorithm::FMMM)) {
            graph.current_layout = LayoutAlgorithm::FMMM;
            Layout::applyOGDFLayout(graph);
            graph.saveLayoutToCache(LayoutAlgorithm::FMMM);
          }
        }
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }

    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(
        ImVec2(viewport->WorkSize.x - 320.0f, viewport->WorkSize.y));

    ImGui::Begin("EDA Netlist Graph Viewer", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImVec2 canvas_p0 = ImGui::GetCursorScreenPos(); // Top-left of canvas
    ImVec2 canvas_p1 = ImVec2(canvas_p0.x + ImGui::GetContentRegionAvail().x,
                              canvas_p0.y + ImGui::GetContentRegionAvail().y);

    ImGuiIO &io = ImGui::GetIO();
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    draw_list->AddRectFilled(canvas_p0, canvas_p1, col_bg);

    // --- Input Handling ---
    bool is_hovered = ImGui::IsWindowHovered();
    bool is_active = ImGui::IsWindowFocused();

    // Panning with Middle Mouse or Double-Left click + Drag
    if (is_hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f)) {
      camera.pan.x += io.MouseDelta.x;
      camera.pan.y += io.MouseDelta.y;
    }

    if (isDraggingEmpty) {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        camera.pan.x += io.MouseDelta.x;
        camera.pan.y += io.MouseDelta.y;
      } else {
        isDraggingEmpty = false; // Stop dragging when mouse released
      }
    }

    if (is_hovered && io.MouseWheel != 0.0f) {
      float zoom_factor = powf(1.1f, io.MouseWheel);
      Vec2 mouse_world = camera.screenToWorld(
          {io.MousePos.x - canvas_p0.x, io.MousePos.y - canvas_p0.y});
      camera.zoom *= zoom_factor;
      Vec2 new_mouse_world = camera.screenToWorld(
          {io.MousePos.x - canvas_p0.x, io.MousePos.y - canvas_p0.y});
      camera.pan.x += (new_mouse_world.x - mouse_world.x) * camera.zoom;
      camera.pan.y += (new_mouse_world.y - mouse_world.y) * camera.zoom;
    }

    Rect screenBounds = {0, 0, canvas_p1.x - canvas_p0.x,
                         canvas_p1.y - canvas_p0.y};

    Rect worldBounds = {
        camera.screenToWorld({screenBounds.x, screenBounds.y}).x,
        camera.screenToWorld({screenBounds.x, screenBounds.y}).y,
        screenBounds.w / camera.zoom, screenBounds.h / camera.zoom};

    // --- Culling & Rendering ---
    std::vector<int> visibleNodes = graph.queryVisibleNodes(worldBounds);
    std::vector<int> visibleEdges = graph.queryVisibleEdges(worldBounds);

    // Handle Interactions on Nodes
    bool mouse_clicked_left = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    bool mouse_clicked_right = ImGui::IsMouseClicked(ImGuiMouseButton_Right);

    if (is_hovered && (mouse_clicked_left || mouse_clicked_right)) {
      if (mouse_clicked_left) {
        selection.clear();
      } // Right click doesn't clear selection unless we hit something else, or
        // maybe it should.

      Vec2 mouse_pos = {io.MousePos.x - canvas_p0.x,
                        io.MousePos.y - canvas_p0.y};
      float hit_radius = 16.0f; // pixel radius for click targets

      bool entityClicked = false;

      // 1. Check Ports (drawn on top of nodes)
      for (int id : visibleNodes) {
        const auto &node = graph.getNode(id);
        for (const auto &port : node.ports) {
          Vec2 portWorldPos = {node.bounds.x + port.local_pos.x,
                               node.bounds.y + port.local_pos.y};
          Vec2 portScreen = camera.worldToScreen(portWorldPos);

          Vec2 diff = {mouse_pos.x - portScreen.x, mouse_pos.y - portScreen.y};
          if (diff.x * diff.x + diff.y * diff.y <= hit_radius * hit_radius) {
            selection.type = SelectionType::Port;
            selection.id = port.id;
            selection.data = (void *)&port.data;
            entityClicked = true;

            if (mouse_clicked_right) {
              ImGui::OpenPopup("PortContextMenu");
            } else if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
              // If it's an input port, we want to toggle all incoming edges.
              // If output port, toggle all outgoing edges.
              bool any_hidden = false;
              for (int edge_id : port.connected_edge_ids) {
                if (!graph.getEdge(edge_id).visible)
                  any_hidden = true;
              }
              for (int edge_id : port.connected_edge_ids) {
                graph.getEdge(edge_id).visible =
                    any_hidden; // Make all visible if any was hidden, else hide
                                // all
              }
            }
            break;
          }
        }
        if (entityClicked)
          break;
      }

      // 2. Check Nodes
      if (!entityClicked) {
        Vec2 mouse_world = camera.screenToWorld(mouse_pos);
        for (int id : visibleNodes) {
          const auto &node = graph.getNode(id);
          if (node.bounds.contains(mouse_world)) {
            selection.type = SelectionType::Node;
            selection.id = id;
            selection.data = (void *)&node.data;
            entityClicked = true;
            break;
          }
        }
      }

      // 3. Check Edges (if no port or node was clicked)
      if (!entityClicked) {
        float edge_hit_req_sq = 100.0f; // 10 pixels squared tolerance
        for (int edge_id : visibleEdges) {
          const auto &edge = graph.getEdge(edge_id);
          if (!edge.visible && !graph.global_hide_all)
            continue;

          bool edgeHit = false;
          for (size_t i = 0; i + 1 < edge.waypoints.size(); ++i) {
            Vec2 p1 = camera.worldToScreen(edge.waypoints[i]);
            Vec2 p2 = camera.worldToScreen(edge.waypoints[i + 1]);
            if (dist_to_segment_squared(p1, p2, mouse_pos) < edge_hit_req_sq) {
              edgeHit = true;
              break;
            }
          }
          if (edgeHit) {
            selection.type = SelectionType::Edge;
            selection.id = edge.id;
            selection.data = (void *)&edge.data;
            entityClicked = true;
            break;
          }
        }
      }

      if (!entityClicked && is_hovered &&
          ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        isDraggingEmpty = true;
      }
    }

    // Render Context Menus
    if (ImGui::BeginPopup("PortContextMenu")) {
      if (selection.type == SelectionType::Port) {
        ImGui::Text("Jump to Connected Node:");
        ImGui::Separator();

        bool has_links = false;
        // Re-find the port
        bool port_found = false;
        const Port<NodeData, EdgeData, PortData> *p = nullptr;
        for (const auto &n : graph.getNodes()) {
          for (const auto &port : n.ports) {
            if (port.id == selection.id) {
              p = &port;
              port_found = true;
              break;
            }
          }
          if (port_found)
            break;
        }

        if (p) {
          for (int edge_id : p->connected_edge_ids) {
            has_links = true;
            const auto &edge = graph.getEdge(edge_id);
            int target_node_id = (p->type == PortType::Output)
                                     ? edge.dst_node_id
                                     : edge.src_node_id;
            std::string label = "Go to Node " + std::to_string(target_node_id);
            if (ImGui::MenuItem(label.c_str())) {
              // Jump Camera
              const auto &target_node = graph.getNode(target_node_id);
              camera.pan.x = -target_node.bounds.x * camera.zoom +
                             (canvas_p1.x - canvas_p0.x) / 2.0f;
              camera.pan.y = -target_node.bounds.y * camera.zoom +
                             (canvas_p1.y - canvas_p0.y) / 2.0f;

              // Select target node
              selection.type = SelectionType::Node;
              selection.id = target_node_id;
              selection.data = (void *)&target_node.data;
            }
          }
        }

        if (!has_links) {
          ImGui::TextDisabled("No connected edges.");
        }
      }
      ImGui::EndPopup();
    }

    // Render Edges
    for (int edge_id : visibleEdges) {
      const auto &edge = graph.getEdge(edge_id);
      if (!edge.visible && !graph.global_hide_all)
        continue;

      ImVec2 points[16];
      int pt_count = 0;
      for (const auto &wp : edge.waypoints) {
        if (pt_count >= 16)
          break;
        Vec2 screenPos = camera.worldToScreen(wp);
        points[pt_count++] =
            ImVec2(canvas_p0.x + screenPos.x, canvas_p0.y + screenPos.y);
      }
      if (pt_count > 1) {
        ImU32 draw_color =
            (selection.type == SelectionType::Edge && selection.id == edge.id)
                ? col_edge_selected
                : col_edge;
        float thickness =
            (selection.type == SelectionType::Edge && selection.id == edge.id)
                ? 4.0f
                : 2.5f;
        draw_list->AddPolyline(points, pt_count, draw_color, 0,
                               thickness * camera.zoom);
      }
    }

    // Render Nodes
    for (int id : visibleNodes) {
      const auto &node = graph.getNode(id);
      Rect screenRect = camera.worldToScreenRect(node.bounds);
      ImVec2 p_min =
          ImVec2(canvas_p0.x + screenRect.x, canvas_p0.y + screenRect.y);
      ImVec2 p_max = ImVec2(p_min.x + screenRect.w, p_min.y + screenRect.h);

      draw_list->AddRectFilled(
          p_min, p_max,
          (selection.type == SelectionType::Node && selection.id == id)
              ? col_node_selected
              : col_node,
          4.0f);
      draw_list->AddRect(p_min, p_max, col_node_border, 4.0f);

      // Draw Ports
      for (const auto &port : node.ports) {
        Vec2 portWorldPos = {node.bounds.x + port.local_pos.x,
                             node.bounds.y + port.local_pos.y};
        Vec2 portScreen = camera.worldToScreen(portWorldPos);
        ImVec2 pt =
            ImVec2(canvas_p0.x + portScreen.x, canvas_p0.y + portScreen.y);
        draw_list->AddCircleFilled(pt, 5.0f * camera.zoom, col_port);
      }
    }

    ImGui::End();
  }

  size_t getVisibleNodeCount() const {
    return graph
        .queryVisibleNodes({camera.screenToWorld({0, 0}).x,
                            camera.screenToWorld({0, 0}).y,
                            ImGui::GetContentRegionAvail().x / camera.zoom,
                            ImGui::GetContentRegionAvail().y / camera.zoom})
        .size();
  }

  // Abstract property rendering depending on the data type using an external
  // callback or specialized function
  void renderPropertiesPanel(
      void (*renderNodeData)(const Node<NodeData, EdgeData, PortData> &,
                             const Graph<NodeData, EdgeData, PortData> &),
      void (*renderEdgeData)(const Edge<NodeData, EdgeData, PortData> &,
                             const Graph<NodeData, EdgeData, PortData> &),
      void (*renderPortData)(const Port<NodeData, EdgeData, PortData> &,
                             const Node<NodeData, EdgeData, PortData> &,
                             const Graph<NodeData, EdgeData, PortData> &),
      size_t visibleNodeCount) {
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 320.0f,
               viewport->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(320.0f, viewport->WorkSize.y));

    ImGui::Begin("Properties", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings);

    if (selection.type == SelectionType::Node && selection.data) {
      const auto &node = graph.getNode(selection.id);
      ImGui::Text("Selected: Node %d", selection.id);
      renderNodeData(node, graph);
    } else if (selection.type == SelectionType::Edge && selection.data) {
      const auto &edge = graph.getEdge(selection.id);
      ImGui::Text("Selected: Edge %d", selection.id);
      renderEdgeData(edge, graph);
    } else if (selection.type == SelectionType::Port && selection.data) {
      // Find owning node to pass it cleanly
      for (const auto &node : graph.getNodes()) {
        for (const auto &port : node.ports) {
          if (port.id == selection.id) {
            ImGui::Text("Selected: Port %d", selection.id);
            renderPortData(port, node, graph);
            break;
          }
        }
      }
    } else {
      ImGui::Text("No entity selected.");
    }

    // Bottom aligned info text
    ImVec2 content_avail = ImGui::GetContentRegionAvail();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + content_avail.y - 60.0f);
    ImGui::Separator();
    ImGui::Text("Render Info:");
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Visible Nodes: %zu", visibleNodeCount);

    ImGui::End();
  }
};
