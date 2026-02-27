// Dear ImGui: standalone example application for SDL3 + SDL_Renderer
// (SDL is a cross-platform general purpose library for handling windows,
// inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/
// folder).
// - Introduction, links and more at the top of imgui.cpp

// Important to understand: SDL_Renderer is an _optional_ component of SDL3.
// For a multi-platform app consider using e.g. SDL+DirectX on Windows and
// SDL+OpenGL on Linux/OSX.

#include "Graph.h"
#include "GraphRenderer.h"
#include "Layout.h"
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"
#include <SDL3/SDL.h>
#include <random>

// Mock User Data Types
struct MyNodeData {
  std::string name;
  std::string cell_type;
};
struct MyEdgeData {
  std::string net_name;
  float capacitance;
};
struct MyPortData {
  std::string pin_name;
};

using GraphType = Graph<MyNodeData, MyEdgeData, MyPortData>;

void RenderNodeProps(const Node<MyNodeData, MyEdgeData, MyPortData> &node,
                     const GraphType &graph) {
  ImGui::Text("Cell Name: %s", node.data.name.c_str());
  ImGui::Text("Cell Type: %s", node.data.cell_type.c_str());

  if (ImGui::TreeNode("Pins")) {
    for (const auto &port : node.ports) {
      ImGui::BulletText("%s (%s)", port.data.pin_name.c_str(),
                        port.type == PortType::Input ? "IN" : "OUT");
    }
    ImGui::TreePop();
  }
}

void RenderEdgeProps(const Edge<MyNodeData, MyEdgeData, MyPortData> &edge,
                     const GraphType &graph) {
  ImGui::Text("Net Name: %s", edge.data.net_name.c_str());
  ImGui::Text("Capacitance: %.2f fF", edge.data.capacitance);

  ImGui::Separator();
  const auto &src_node = graph.getNode(edge.src_node_id);
  const auto &sink_node = graph.getNode(edge.dst_node_id);
  ImGui::Text("Source Cell: %s", src_node.data.name.c_str());
  ImGui::Text("Sink Cell:   %s", sink_node.data.name.c_str());
}

void RenderPortProps(const Port<MyNodeData, MyEdgeData, MyPortData> &port,
                     const Node<MyNodeData, MyEdgeData, MyPortData> &owner,
                     const GraphType &graph) {
  ImGui::Text("Pin Name: %s", port.data.pin_name.c_str());
  ImGui::Text("Direction: %s",
              port.type == PortType::Input ? "Input" : "Output");
  ImGui::Text("Owning Cell: %s", owner.data.name.c_str());

  if (!port.connected_edge_ids.empty()) {
    ImGui::Separator();
    ImGui::Text("Connected Nets:");
    for (int eid : port.connected_edge_ids) {
      const auto &edge = graph.getEdge(eid);
      const auto &src_node = graph.getNode(edge.src_node_id);
      ImGui::BulletText("%s (from %s)", edge.data.net_name.c_str(),
                        src_node.data.name.c_str());
    }
  }
}

// ... inside main()

int main(int argc, char **argv) {
  // Setup SDL
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    printf("Error: SDL_Init(): %s\n", SDL_GetError());
    return 1;
  }

  float main_scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
  SDL_WindowFlags window_flags =
      SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY;
  SDL_Window *window =
      SDL_CreateWindow("EDA Netlist Analyzer", (int)(1280 * main_scale),
                       (int)(800 * main_scale), window_flags);
  if (window == nullptr)
    return 1;

  SDL_Renderer *renderer = SDL_CreateRenderer(window, nullptr);
  SDL_SetRenderVSync(renderer, 1);
  SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
  SDL_ShowWindow(window);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  ImGui::StyleColorsDark();
  ImGuiStyle &style = ImGui::GetStyle();
  style.ScaleAllSizes(main_scale);
  style.FontScaleDpi = main_scale;

  ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer3_Init(renderer);

  ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.1f, 1.00f);

  // Initialize Graph with Dummy Data
  Graph<MyNodeData, MyEdgeData, MyPortData> graph;

  printf("Loading Graph Data...\n");
  int num_nodes = (argc == 1) ? 1000 : atoi(argv[1]);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib(0, num_nodes - 1);

  for (int i = 0; i < num_nodes; i++) {
    int node =
        graph.addNode({0, 0, 100, 50}, {"U" + std::to_string(i), "AND2"});
    graph.addPort(node, PortType::Input, {0, 10}, {"A"});
    graph.addPort(node, PortType::Input, {0, 30}, {"B"});
    graph.addPort(node, PortType::Output, {100, 20}, {"Y"});
  }

  // Connect random nets
  for (int i = 0; i < num_nodes; i++) {
    int target = distrib(gen);
    if (i != target) {
      // Find an output port on source and input port on dest
      int src_port = -1, dst_port = -1;
      for (auto &p : graph.getNode(i).ports) {
        if (p.type == PortType::Output) {
          src_port = p.id;
          break;
        }
      }
      for (auto &p : graph.getNode(target).ports) {
        if (p.type == PortType::Input) {
          dst_port = p.id;
          break;
        }
      }

      if (src_port != -1 && dst_port != -1) {
        graph.addEdge(i, src_port, target, dst_port,
                      {"n" + std::to_string(i), 10.5f});
      }
    }
  }

  printf("Applying Layout...\n");
  // Layout::applyOGDFLayout(graph);
  // graph.rebuildQuadtree();

  GraphRenderer<MyNodeData, MyEdgeData, MyPortData> graphRenderer(graph);

  // 1. When starting up put view where at least some rectangles are there
  if (!graph.getNodes().empty()) {
    graphRenderer.triggerLayout(LayoutAlgorithm::FastHierarchy);
    const auto &firstNode = graph.getNodes()[0];
    // We can't access camera directly from main unless exposed, so we'll do it
    // inside render loop implicitly as added earlier in "Reset View".
    // Alternatively, just trigger a reset view logic here.
  }

  bool done = false;
  while (!done) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL3_ProcessEvent(&event);
      if (event.type == SDL_EVENT_QUIT)
        done = true;
      if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED &&
          event.window.windowID == SDL_GetWindowID(window))
        done = true;
    }

    if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) {
      SDL_Delay(10);
      continue;
    }

    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    // Fullscreen Dockspace
    // Docking requires docking branch of ImGui, removing for now to resolve
    // lints

    // Render our app
    graphRenderer.render();
    graphRenderer.renderPropertiesPanel(RenderNodeProps, RenderEdgeProps,
                                        RenderPortProps,
                                        graphRenderer.getVisibleNodeCount());

    ImGui::Render();
    SDL_SetRenderScale(renderer, io.DisplayFramebufferScale.x,
                       io.DisplayFramebufferScale.y);
    SDL_SetRenderDrawColorFloat(renderer, clear_color.x, clear_color.y,
                                clear_color.z, clear_color.w);
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
  }

  ImGui_ImplSDLRenderer3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
