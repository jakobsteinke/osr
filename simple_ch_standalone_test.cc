#include <chrono>
#include <random>
#include <iostream>
#include <vector>
#include <queue>

#include "include/osr/routing/simple_ch_graph.h"
#include "include/osr/routing/simple_ch_preprocessor.h"
#include "include/osr/routing/simple_ch_query.h"
#include "include/osr/routing/dial.h"

namespace osr {

// Simple Dijkstra implementation for comparison using dial queue
class simple_dijkstra {
public:
  explicit simple_dijkstra(const simple_ch_graph& graph)
    : graph_(graph),
      node_count_(graph.node_count()),
      distance_(node_count_, kInvalidWeight),
      predecessor_(node_count_, kInvalidNode),
      pq_(get_bucket{}) {}

  query_result query(unsigned source, unsigned target, unsigned max_cost = kInvalidWeight) {
    reset(max_cost);
    
    if (source >= node_count_ || target >= node_count_) {
      return {};
    }
    
    if (source == target) {
      return {0, {source}};
    }
    
    distance_[source] = 0;
    pq_.push(ch_label{source, 0});
    
    while (!pq_.empty()) {
      auto current = pq_.pop();
      unsigned node = current.node;
      unsigned dist = current.cost;
      
      if (dist > distance_[node]) continue;
      
      if (node == target) {
        return {dist, reconstruct_path(source, target)};
      }
      
      for (const auto& arc : graph_.out_arcs(node)) {
        unsigned neighbor = arc.node;
        unsigned new_dist = dist + arc.weight;
        
        if (new_dist < distance_[neighbor] && new_dist < kInvalidWeight) {
          distance_[neighbor] = new_dist;
          predecessor_[neighbor] = node;
          pq_.push(ch_label{neighbor, new_dist});
        }
      }
    }
    
    return {};  // No path found
  }

private:
  void reset(unsigned max_cost = kInvalidWeight) {
    std::fill(distance_.begin(), distance_.end(), kInvalidWeight);
    std::fill(predecessor_.begin(), predecessor_.end(), kInvalidNode);
    pq_.clear();
    if (max_cost != kInvalidWeight && max_cost > 0) {
      pq_.n_buckets(max_cost + 1U);
    } else {
      pq_.n_buckets(10000U);
    }
  }

  std::vector<unsigned> reconstruct_path(unsigned source, unsigned target) {
    std::vector<unsigned> path;
    unsigned current = target;
    
    while (current != source) {
      path.push_back(current);
      current = predecessor_[current];
      if (current == kInvalidNode) break;
    }
    path.push_back(source);
    std::reverse(path.begin(), path.end());
    return path;
  }

  const simple_ch_graph& graph_;
  unsigned node_count_;
  std::vector<unsigned> distance_;
  std::vector<unsigned> predecessor_;
  dial<ch_label, get_bucket> pq_;
};

// Helper function to create a simple grid graph for testing
simple_ch_graph create_grid_graph(unsigned width, unsigned height) {
  unsigned node_count = width * height;
  simple_ch_graph graph(node_count);
  
  auto node_id = [width](unsigned x, unsigned y) { return y * width + x; };
  
  // Add edges for grid connectivity
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      unsigned current = node_id(x, y);
      
      // Right edge
      if (x + 1 < width) {
        graph.add_edge(current, node_id(x + 1, y), 1);
      }
      
      // Down edge  
      if (y + 1 < height) {
        graph.add_edge(current, node_id(x, y + 1), 1);
      }
      
      // Left edge
      if (x > 0) {
        graph.add_edge(current, node_id(x - 1, y), 1);
      }
      
      // Up edge
      if (y > 0) {
        graph.add_edge(current, node_id(x, y - 1), 1);
      }
    }
  }
  
  return graph;
}

}  // namespace osr

int main() {
  // Create a reasonably sized grid graph
  constexpr unsigned WIDTH = 100;
  constexpr unsigned HEIGHT = 100; 
  constexpr unsigned NUM_QUERIES = 100;
  
  std::cout << "Creating " << WIDTH << "x" << HEIGHT << " grid graph...\n";
  auto graph = osr::create_grid_graph(WIDTH, HEIGHT);
  
  std::cout << "Running CH preprocessing...\n";
  osr::simple_ch_preprocessor preprocessor(graph);
  auto start_prep = std::chrono::high_resolution_clock::now();
  preprocessor.preprocess();
  auto end_prep = std::chrono::high_resolution_clock::now();
  
  auto prep_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_prep - start_prep);
  std::cout << "CH preprocessing took: " << prep_time.count() << " ms\n";
  
  // Create algorithm instances
  osr::simple_dijkstra dijkstra(graph);
  osr::simple_ch_query ch_query(graph);
  
  // Generate random query pairs
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<unsigned> dis(0, graph.node_count() - 1);
  
  std::vector<std::pair<unsigned, unsigned>> queries;
  for (unsigned i = 0; i < NUM_QUERIES; ++i) {
    unsigned source = dis(gen);
    unsigned target = dis(gen);
    if (source != target) {
      queries.emplace_back(source, target);
    }
  }
  
  std::cout << "Running " << queries.size() << " queries with both algorithms...\n";
  
  constexpr unsigned MAX_COST = 1000;  // Reasonable max for grid graph
  
  // Measure Dijkstra performance
  auto start_dijkstra = std::chrono::high_resolution_clock::now();
  std::vector<osr::query_result> dijkstra_results;
  for (const auto& [source, target] : queries) {
    dijkstra_results.push_back(dijkstra.query(source, target, MAX_COST));
  }
  auto end_dijkstra = std::chrono::high_resolution_clock::now();
  
  // Measure CH performance
  auto start_ch = std::chrono::high_resolution_clock::now();
  std::vector<osr::query_result> ch_results;
  for (const auto& [source, target] : queries) {
    ch_results.push_back(ch_query.query(source, target, MAX_COST));
  }
  auto end_ch = std::chrono::high_resolution_clock::now();
  
  auto dijkstra_time = std::chrono::duration_cast<std::chrono::microseconds>(end_dijkstra - start_dijkstra);
  auto ch_time = std::chrono::duration_cast<std::chrono::microseconds>(end_ch - start_ch);
  
  // Verify correctness - distances should match
  unsigned correct_queries = 0;
  for (size_t i = 0; i < queries.size(); ++i) {
    if (dijkstra_results[i].distance == ch_results[i].distance) {
      correct_queries++;
    } else {
      std::cout << "Distance mismatch for query " << i << ": Dijkstra=" 
                << dijkstra_results[i].distance << ", CH=" << ch_results[i].distance << "\n";
    }
  }
  
  std::cout << "\n=== PERFORMANCE RESULTS ===\n";
  std::cout << "Dijkstra total time: " << dijkstra_time.count() << " μs\n";
  std::cout << "CH total time:       " << ch_time.count() << " μs\n";
  std::cout << "Dijkstra avg/query:  " << (dijkstra_time.count() / queries.size()) << " μs\n";
  std::cout << "CH avg/query:        " << (ch_time.count() / queries.size()) << " μs\n";
  
  if (ch_time.count() > 0) {
    double speedup = static_cast<double>(dijkstra_time.count()) / ch_time.count();
    std::cout << "Speedup:             " << speedup << "x\n";
  }
  
  std::cout << "Correctness:         " << correct_queries << "/" << queries.size() 
            << " (" << (100.0 * correct_queries / queries.size()) << "%)\n";
  
  bool success = (correct_queries > queries.size() * 0.95);  // At least 95% correct
  std::cout << "Test result:         " << (success ? "PASSED" : "FAILED") << "\n";
  
  return success ? 0 : 1;
}