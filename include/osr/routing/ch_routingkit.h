#pragma once

#include <vector>
#include <limits>
#include <functional>
#include <cassert>
#include "osr/types.h"

namespace osr {

/**
 * RoutingKit-compatible ContractionHierarchy data structure.
 * Implements the same interface and algorithms as RoutingKit for maximum compatibility.
 */
class ContractionHierarchy {
public:
    static constexpr std::uint32_t INVALID_ID = std::numeric_limits<std::uint32_t>::max();
    static constexpr std::uint32_t INF_WEIGHT = std::numeric_limits<std::uint32_t>::max();
    
    struct Side {
        std::vector<std::uint32_t> first_out;  // first_out[node] = index of first outgoing arc
        std::vector<std::uint32_t> head;       // head[arc] = target node of arc
        std::vector<std::uint32_t> weight;     // weight[arc] = cost of arc
    };
    
    // Build CH from simple directed graph
    static ContractionHierarchy build(
        std::uint32_t node_count,
        std::vector<std::uint32_t> tail,
        std::vector<std::uint32_t> head,
        std::vector<std::uint32_t> weight,
        std::function<void(std::string const&)> log_message = nullptr
    );
    
    std::uint32_t node_count() const {
        return rank.size();
    }
    
    std::vector<std::uint32_t> rank;   // rank[node] = contraction order (0 = first contracted)
    std::vector<std::uint32_t> order;  // order[rank] = node (inverse of rank)
    Side forward;   // Forward graph (upward edges only)  
    Side backward;  // Backward graph (upward edges only)
    
private:
    ContractionHierarchy() = default;
};

/**
 * RoutingKit-compatible bidirectional query on ContractionHierarchy.
 * Implements the same interface and algorithms as RoutingKit.
 */
class ContractionHierarchyQuery {
public:
    ContractionHierarchyQuery() : ch_(nullptr) {}
    explicit ContractionHierarchyQuery(ContractionHierarchy const& ch);
    
    ContractionHierarchyQuery& reset();
    ContractionHierarchyQuery& reset(ContractionHierarchy const& ch);
    
    ContractionHierarchyQuery& add_source(std::uint32_t s, std::uint32_t dist_to_s = 0);
    ContractionHierarchyQuery& add_target(std::uint32_t t, std::uint32_t dist_to_t = 0);
    
    ContractionHierarchyQuery& run();
    
    std::uint32_t get_distance() const;
    std::vector<std::uint32_t> get_node_path();
    
private:
    enum class State {
        Initialized,
        Run
    } state_ = State::Initialized;
    
    ContractionHierarchy const* ch_;
    
    // Priority queue implementation
    struct QueueEntry {
        std::uint32_t node;
        std::uint32_t dist;
        
        bool operator>(QueueEntry const& other) const {
            return dist > other.dist;
        }
    };
    
    std::vector<QueueEntry> forward_queue_, backward_queue_;
    std::vector<bool> forward_pushed_, backward_pushed_;
    std::vector<std::uint32_t> forward_dist_, backward_dist_;
    std::vector<std::uint32_t> forward_pred_, backward_pred_;
    
    std::uint32_t shortest_path_length_;
    std::uint32_t meeting_node_;
    
    void make_heap(std::vector<QueueEntry>& queue);
    void push_heap(std::vector<QueueEntry>& queue, QueueEntry entry);
    QueueEntry pop_heap(std::vector<QueueEntry>& queue);
    bool empty(std::vector<QueueEntry> const& queue) const;
    QueueEntry const& top(std::vector<QueueEntry> const& queue) const;
    
    template<bool IsForward>
    void settle_node();
    
    template<bool IsForward>
    void expand_node(std::uint32_t node, std::uint32_t dist);
    
    bool can_stall(std::uint32_t node, std::uint32_t dist, bool is_forward) const;
};

}  // namespace osr