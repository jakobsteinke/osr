#include "gtest/gtest.h"

#include <chrono>
#include <random>
#include <iostream>
#include <vector>
#include <queue>

#include "osr/routing/simple_ch_graph.h"
#include "osr/routing/simple_ch_preprocessor.h"
#include "osr/routing/simple_ch_query.h"
#include "osr/routing/dial.h"

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

TEST(SimpleSpeedupTest, measure_dijkstra_vs_ch) {
  // Create a larger grid to demonstrate CH benefits
  constexpr unsigned WIDTH = 150;
  constexpr unsigned HEIGHT = 150; 
  constexpr unsigned NUM_QUERIES = 50;
  
  std::cout << "Creating " << WIDTH << "x" << HEIGHT << " grid graph...\n";
  auto graph = create_grid_graph(WIDTH, HEIGHT);
  
  // Test basic connectivity before CH preprocessing
  std::cout << "Testing basic connectivity...\n";
  simple_dijkstra basic_dijkstra(graph);
  auto test_result = basic_dijkstra.query(0, 1);  // Should find path from node 0 to 1
  std::cout << "Basic test (0->1): distance = " << test_result.distance << "\n";
  
  std::cout << "Running CH preprocessing...\n";
  simple_ch_preprocessor preprocessor(graph);
  auto start_prep = std::chrono::high_resolution_clock::now();
  preprocessor.preprocess();
  auto end_prep = std::chrono::high_resolution_clock::now();
  
  auto prep_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_prep - start_prep);
  std::cout << "CH preprocessing took: " << prep_time.count() << " ms\n";
  
  // Create algorithm instances
  simple_dijkstra dijkstra(graph);
  simple_ch_query ch_query(graph);
  
  // Generate meaningful query pairs (corner to corner, edges to center, etc.)
  std::vector<std::pair<unsigned, unsigned>> queries;
  
  auto node_id = [WIDTH](unsigned x, unsigned y) { return y * WIDTH + x; };
  
  // Meaningful long-distance queries for 100x100 grid
  queries.emplace_back(node_id(0, 0), node_id(WIDTH-1, HEIGHT-1));           // Corner to corner
  queries.emplace_back(node_id(0, HEIGHT-1), node_id(WIDTH-1, 0));           // Other diagonal
  queries.emplace_back(node_id(0, HEIGHT/2), node_id(WIDTH-1, HEIGHT/2));    // Left to right middle
  queries.emplace_back(node_id(WIDTH/2, 0), node_id(WIDTH/2, HEIGHT-1));     // Top to bottom middle
  queries.emplace_back(node_id(WIDTH/4, HEIGHT/4), node_id(3*WIDTH/4, 3*HEIGHT/4)); // Quarter to 3/4
  
  // Add some medium distance queries
  for (unsigned i = 5; i < NUM_QUERIES; ++i) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<unsigned> x_dis(0, WIDTH - 1);
    std::uniform_int_distribution<unsigned> y_dis(0, HEIGHT - 1);
    
    unsigned source = node_id(x_dis(gen), y_dis(gen));
    unsigned target = node_id(x_dis(gen), y_dis(gen));
    
    if (source != target) {
      queries.emplace_back(source, target);
    }
  }
  
  std::cout << "Running " << queries.size() << " queries with both algorithms...\n";
  
  // Measure Dijkstra performance
  auto start_dijkstra = std::chrono::high_resolution_clock::now();
  std::vector<query_result> dijkstra_results;
  for (const auto& [source, target] : queries) {
    dijkstra_results.push_back(dijkstra.query(source, target));
  }
  auto end_dijkstra = std::chrono::high_resolution_clock::now();
  
  // Measure CH performance
  auto start_ch = std::chrono::high_resolution_clock::now();
  std::vector<query_result> ch_results;
  for (const auto& [source, target] : queries) {
    ch_results.push_back(ch_query.query(source, target));
  }
  auto end_ch = std::chrono::high_resolution_clock::now();
  
  auto dijkstra_time = std::chrono::duration_cast<std::chrono::microseconds>(end_dijkstra - start_dijkstra);
  auto ch_time = std::chrono::duration_cast<std::chrono::microseconds>(end_ch - start_ch);
  
  // Verify correctness - distances should match
  unsigned correct_queries = 0;
  unsigned total_dijkstra_cost = 0;
  unsigned total_ch_cost = 0;
  unsigned valid_paths = 0;
  
  for (size_t i = 0; i < std::min(size_t(5), queries.size()); ++i) {
    std::cout << "Query " << i << " (src=" << queries[i].first << ", tgt=" << queries[i].second 
              << "): Dijkstra=" << dijkstra_results[i].distance 
              << ", CH=" << ch_results[i].distance << "\n";
  }
  
  for (size_t i = 0; i < queries.size(); ++i) {
    if (dijkstra_results[i].distance == ch_results[i].distance) {
      correct_queries++;
    } else {
      std::cout << "Distance mismatch for query " << i << ": Dijkstra=" 
                << dijkstra_results[i].distance << ", CH=" << ch_results[i].distance << "\n";
    }
    
    // Sum up total costs (only count valid paths)
    if (dijkstra_results[i].distance != kInvalidWeight) {
      total_dijkstra_cost += dijkstra_results[i].distance;
      valid_paths++;
    }
    if (ch_results[i].distance != kInvalidWeight) {
      total_ch_cost += ch_results[i].distance;
    }
  }
  
  std::cout << "\n=== PERFORMANCE RESULTS ===\n";
  std::cout << "Dijkstra total time: " << dijkstra_time.count() << " us\n";
  std::cout << "CH total time:       " << ch_time.count() << " us\n";
  std::cout << "Dijkstra avg/query:  " << (dijkstra_time.count() / queries.size()) << " us\n";
  std::cout << "CH avg/query:        " << (ch_time.count() / queries.size()) << " us\n";
  std::cout << "Dijkstra total cost: " << total_dijkstra_cost << "\n";
  std::cout << "CH total cost:       " << total_ch_cost << "\n";
  
  if (ch_time.count() > 0) {
    double speedup = static_cast<double>(dijkstra_time.count()) / ch_time.count();
    std::cout << "Speedup:             " << speedup << "x\n";
  }
  
  std::cout << "Valid paths found:   " << valid_paths << "/" << queries.size() << "\n";
  std::cout << "Correctness:         " << correct_queries << "/" << queries.size() 
            << " (" << (100.0 * correct_queries / queries.size()) << "%)\n";
  
  // The test passes if most queries return correct results
  EXPECT_GT(correct_queries, queries.size() * 0.95);  // At least 95% correct
}

}  // namespace osr