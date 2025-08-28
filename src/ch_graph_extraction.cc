#include "osr/routing/ch_graph_extraction.h"
#include "osr/routing/profiles/car_no_restrictions.h"
#include <iostream>

namespace osr {

void extract_simple_graph(
    ways const& w,
    std::vector<std::uint32_t>& tail,
    std::vector<std::uint32_t>& head, 
    std::vector<std::uint32_t>& weight) {
    
    tail.clear();
    head.clear();
    weight.clear();
    
    std::cout << "Extracting simple graph from OSR with " << w.n_nodes() << " nodes...\n";
    
    // Reserve space for efficiency
    tail.reserve(w.n_ways() * 4);  // Rough estimate
    head.reserve(w.n_ways() * 4);
    weight.reserve(w.n_ways() * 4);
    
    std::uint32_t edge_count = 0;
    
    // Iterate through all nodes and extract their outgoing edges using car_no_restrictions
    for (node_idx_t node_idx{0}; node_idx.v_ < w.n_nodes(); ++node_idx.v_) {
        
        // Get all car::node instances for this node_idx
        car_no_restrictions::resolve_all(*w.r_, node_idx, level_t{0.0F}, 
            [&](car_no_restrictions::node const& from_node) {
                
                // Use car_no_restrictions::adjacent to get all outgoing edges
                car_no_restrictions::template adjacent<direction::kForward, false>(
                    *w.r_, from_node, nullptr, nullptr, nullptr,
                    [&](car_no_restrictions::node const& to_node, 
                        std::uint32_t const cost, 
                        distance_t const /*dist*/,
                        way_idx_t const /*way*/, 
                        std::uint16_t const /*from_pos*/, 
                        std::uint16_t const /*to_pos*/,
                        elevation_storage::elevation const /*elevation*/, 
                        bool const /*uses_elevator*/) {
                        
                        // Add edge to simple graph
                        tail.push_back(from_node.get_node().v_);
                        head.push_back(to_node.get_node().v_);
                        weight.push_back(cost);
                        ++edge_count;
                    });
            });
            
        // Progress reporting
        if (node_idx.v_ % 1000 == 0 && node_idx.v_ > 0) {
            std::cout << "Processed " << node_idx.v_ << "/" << w.n_nodes() 
                      << " nodes, extracted " << edge_count << " edges\n";
        }
    }
    
    std::cout << "Graph extraction complete: " << w.n_nodes() << " nodes, " 
              << edge_count << " edges\n";
}

}  // namespace osr