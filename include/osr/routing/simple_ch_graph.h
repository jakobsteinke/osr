#pragma once

#include <vector>
#include <limits>
#include <cassert>

namespace osr {

constexpr unsigned kInvalidNode = std::numeric_limits<unsigned>::max();
constexpr unsigned kInvalidWeight = std::numeric_limits<unsigned>::max();

struct simple_arc {
  unsigned node;
  unsigned weight;
  unsigned mid_node = kInvalidNode;  // For shortcuts, stores the bypassed node
};

// Simple CH graph structure based on RoutingKit's design
class simple_ch_graph {
public:
  simple_ch_graph() = default;
  
  explicit simple_ch_graph(unsigned node_count) 
    : node_count_(node_count),
      out_(node_count),
      in_(node_count),
      level_(node_count, 0) {}

  void add_edge(unsigned from, unsigned to, unsigned weight) {
    assert(from < node_count_ && to < node_count_);
    if (from != to) {
      out_[from].push_back({to, weight});
      in_[to].push_back({from, weight});
    }
  }

  void add_shortcut(unsigned from, unsigned to, unsigned weight, unsigned mid_node) {
    assert(from < node_count_ && to < node_count_);
    assert(from != to);
    
    // Check if arc already exists and update weight if shorter
    auto reduce_if_exists = [&](std::vector<simple_arc>& from_list, std::vector<simple_arc>& to_list, unsigned target) -> bool {
      for (auto& arc : from_list) {
        if (arc.node == target) {
          if (arc.weight > weight) {
            arc.weight = weight;
            arc.mid_node = mid_node;
            // Update corresponding reverse arc
            for (auto& rev_arc : to_list) {
              if (rev_arc.node == from) {
                rev_arc.weight = weight;
                rev_arc.mid_node = mid_node;
                break;
              }
            }
          }
          return true;
        }
      }
      return false;
    };

    if (!reduce_if_exists(out_[from], in_[to], to)) {
      out_[from].push_back({to, weight, mid_node});
      in_[to].push_back({from, weight, mid_node});
    }
  }

  void remove_node_edges(unsigned node) {
    assert(node < node_count_);
    
    // Remove outgoing edges from in-lists of targets
    for (const auto& arc : out_[node]) {
      auto& in_list = in_[arc.node];
      in_list.erase(
        std::remove_if(in_list.begin(), in_list.end(),
                      [node](const simple_arc& a) { return a.node == node; }),
        in_list.end());
    }
    
    // Remove incoming edges from out-lists of sources  
    for (const auto& arc : in_[node]) {
      auto& out_list = out_[arc.node];
      out_list.erase(
        std::remove_if(out_list.begin(), out_list.end(),
                      [node](const simple_arc& a) { return a.node == node; }),
        out_list.end());
    }
    
    out_[node].clear();
    in_[node].clear();
  }

  // Getters
  unsigned node_count() const { return node_count_; }
  
  const std::vector<simple_arc>& out_arcs(unsigned node) const {
    assert(node < node_count_);
    return out_[node];
  }
  
  const std::vector<simple_arc>& in_arcs(unsigned node) const {
    assert(node < node_count_);
    return in_[node];
  }
  
  unsigned out_degree(unsigned node) const {
    assert(node < node_count_);
    return out_[node].size();
  }
  
  unsigned in_degree(unsigned node) const {
    assert(node < node_count_);
    return in_[node].size();
  }
  
  unsigned level(unsigned node) const {
    assert(node < node_count_);
    return level_[node];
  }
  
  void set_level(unsigned node, unsigned level) {
    assert(node < node_count_);
    level_[node] = level;
  }

  void raise_level(unsigned node, unsigned level) {
    assert(node < node_count_);
    if (level > level_[node]) {
      level_[node] = level;
    }
  }

private:
  unsigned node_count_ = 0;
  std::vector<std::vector<simple_arc>> out_;
  std::vector<std::vector<simple_arc>> in_;
  std::vector<unsigned> level_;
};

}  // namespace osr