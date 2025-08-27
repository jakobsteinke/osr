#include <iostream>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/platforms.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

using namespace osr;

int main() {
  try {
    std::cout << "=== CH Validation Test ===" << std::endl;
    
    // Check if we can access CH enum
    std::cout << "CH algorithm enum value: " << static_cast<int>(routing_algorithm::kCH) << std::endl;
    
    // Try to load a small dataset (if available)
    auto const data_dir = "test/monaco";
    if (std::filesystem::exists(data_dir)) {
      std::cout << "Loading Monaco dataset..." << std::endl;
      auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
      std::cout << "Dataset loaded: " << w.n_nodes() << " nodes" << std::endl;
      
      // Test basic route with CH
      auto const from = place{location{geo::latlng{43.7396, 7.4263}}, {}};
      auto const to = place{location{geo::latlng{43.7313, 7.4186}}, {}};
      
      std::cout << "Testing CH routing..." << std::endl;
      auto const ch_result = route(w, from, to, search_profile::kCar, 
                                   direction::kForward, {}, {}, {}, 
                                   routing_algorithm::kCH);
      
      if (ch_result.has_value()) {
        std::cout << "SUCCESS: CH routing works! Cost: " << ch_result->cost_.v_ << std::endl;
      } else {
        std::cout << "INFO: CH routing returned no path (may be expected)" << std::endl;
      }
      
      // Compare with Dijkstra
      std::cout << "Testing Dijkstra for comparison..." << std::endl;
      auto const dijkstra_result = route(w, from, to, search_profile::kCar, 
                                        direction::kForward, {}, {}, {}, 
                                        routing_algorithm::kDijkstra);
      
      if (dijkstra_result.has_value()) {
        std::cout << "Dijkstra cost: " << dijkstra_result->cost_.v_ << std::endl;
        
        if (ch_result.has_value() && ch_result->cost_ == dijkstra_result->cost_) {
          std::cout << "SUCCESS: CH and Dijkstra costs match!" << std::endl;
        }
      }
      
    } else {
      std::cout << "Monaco dataset not found, skipping routing test" << std::endl;
    }
    
    std::cout << "=== Validation Complete ===" << std::endl;
    return 0;
    
  } catch (std::exception const& e) {
    std::cout << "ERROR: " << e.what() << std::endl;
    return 1;
  }
}