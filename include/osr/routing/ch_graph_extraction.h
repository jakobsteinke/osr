#pragma once

#include <vector>
#include "osr/ways.h"
#include "osr/routing/profiles/car_no_restrictions.h"
#include "osr/types.h"

namespace osr {

/**
 * Extract a simple directed graph from OSR data structure.
 * Converts OSR's complex way-based representation to a simple (tail, head, weight) format
 * suitable for RoutingKit-style CH preprocessing.
 */
void extract_simple_graph(
    ways const& w,
    std::vector<std::uint32_t>& tail,
    std::vector<std::uint32_t>& head, 
    std::vector<std::uint32_t>& weight
);

}  // namespace osr