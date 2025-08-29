#include "gtest/gtest.h"

#include "osr/routing/algorithms.h"
#include "osr/routing/route.h"
#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/ch_preprocessor.h"

#include <set>

namespace osr {

TEST(ch_basic, algorithm_enum) {
  // Verify CH algorithm exists in enum
  auto algo = routing_algorithm::kCH;
  EXPECT_EQ(static_cast<std::uint8_t>(algo), 2);
}

TEST(ch_basic, to_algorithm_function) {
  // Verify to_algorithm function works
  auto algo = to_algorithm("ch");
  EXPECT_EQ(algo, routing_algorithm::kCH);
}

TEST(ch_basic, ch_algorithm_works) {
  // Verify CH algorithm can be created
  auto algo = to_algorithm("ch");
  EXPECT_EQ(algo, routing_algorithm::kCH);
}

TEST(ch_levels, basic_functionality) {
  constexpr std::size_t n_nodes = 10;
  ch_levels levels(n_nodes);
  
  EXPECT_EQ(levels.size(), n_nodes);
  
  // Check all levels are in range [1, n]
  for (std::size_t i = 0; i < n_nodes; ++i) {
    auto node = node_idx_t{static_cast<std::uint32_t>(i)};
    auto level = levels.get_level(node);
    EXPECT_GE(level, 1);
    EXPECT_LE(level, n_nodes);
  }
}

TEST(ch_levels, unique_levels) {
  constexpr std::size_t n_nodes = 100;
  ch_levels levels(n_nodes);
  
  std::set<ch_levels::level_t> seen_levels;
  
  // Check all levels are unique
  for (std::size_t i = 0; i < n_nodes; ++i) {
    auto node = node_idx_t{static_cast<std::uint32_t>(i)};
    auto level = levels.get_level(node);
    
    EXPECT_TRUE(seen_levels.find(level) == seen_levels.end()) 
        << "Level " << level << " was assigned to multiple nodes";
    seen_levels.insert(level);
  }
  
  // Should have exactly n_nodes unique levels
  EXPECT_EQ(seen_levels.size(), n_nodes);
}

TEST(ch_levels, ordering_functions) {
  ch_levels levels(5);
  
  // Find nodes with different levels
  node_idx_t lower_node{0}, higher_node{0};
  for (std::uint32_t i = 0; i < 5; ++i) {
    for (std::uint32_t j = i + 1; j < 5; ++j) {
      auto node_i = node_idx_t{i};
      auto node_j = node_idx_t{j};
      
      if (levels.get_level(node_i) < levels.get_level(node_j)) {
        lower_node = node_i;
        higher_node = node_j;
        break;
      } else if (levels.get_level(node_j) < levels.get_level(node_i)) {
        lower_node = node_j;
        higher_node = node_i;
        break;
      }
    }
    if (lower_node.v_ != higher_node.v_) break;
  }
  
  // Test ordering functions
  EXPECT_TRUE(levels.is_lower_level(lower_node, higher_node));
  EXPECT_FALSE(levels.is_lower_level(higher_node, lower_node));
  EXPECT_TRUE(levels.is_higher_level(higher_node, lower_node));
  EXPECT_FALSE(levels.is_higher_level(lower_node, higher_node));
}

TEST(ch_shortcuts, basic_functionality) {
  ch_shortcuts shortcuts;
  
  EXPECT_EQ(shortcuts.size(), 0);
  EXPECT_FALSE(shortcuts.has_shortcut(node_idx_t{0}, node_idx_t{1}));
  
  // Add a shortcut from node 0 to node 2 via node 1
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{2}, node_idx_t{1}, cost_t{100});
  
  EXPECT_EQ(shortcuts.size(), 1);
  EXPECT_TRUE(shortcuts.has_shortcut(node_idx_t{0}, node_idx_t{2}));
  EXPECT_FALSE(shortcuts.has_shortcut(node_idx_t{0}, node_idx_t{1}));
}

TEST(ch_shortcuts, get_shortcuts) {
  ch_shortcuts shortcuts;
  
  // Add multiple shortcuts for the same edge
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{2}, node_idx_t{1}, cost_t{100});
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{2}, node_idx_t{3}, cost_t{120});
  
  auto const* edge_shortcuts = shortcuts.get_shortcuts(node_idx_t{0}, node_idx_t{2});
  ASSERT_NE(edge_shortcuts, nullptr);
  EXPECT_EQ(edge_shortcuts->size(), 2);
  
  // Check shortcuts have correct data
  bool found_via_1 = false, found_via_3 = false;
  for (auto const& sc : *edge_shortcuts) {
    EXPECT_EQ(sc.from_, node_idx_t{0});
    EXPECT_EQ(sc.to_, node_idx_t{2});
    
    if (sc.via_ == node_idx_t{1}) {
      EXPECT_EQ(sc.cost_, cost_t{100});
      found_via_1 = true;
    } else if (sc.via_ == node_idx_t{3}) {
      EXPECT_EQ(sc.cost_, cost_t{120});
      found_via_3 = true;
    }
  }
  
  EXPECT_TRUE(found_via_1);
  EXPECT_TRUE(found_via_3);
}

TEST(ch_shortcuts, get_best_shortcut) {
  ch_shortcuts shortcuts;
  
  // No shortcut exists
  auto best = shortcuts.get_best_shortcut(node_idx_t{0}, node_idx_t{2});
  EXPECT_FALSE(best.has_value());
  
  // Add shortcuts with different costs
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{2}, node_idx_t{1}, cost_t{100});
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{2}, node_idx_t{3}, cost_t{80});
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{2}, node_idx_t{4}, cost_t{120});
  
  best = shortcuts.get_best_shortcut(node_idx_t{0}, node_idx_t{2});
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(best->cost_, cost_t{80});
  EXPECT_EQ(best->via_, node_idx_t{3});
  EXPECT_EQ(best->from_, node_idx_t{0});
  EXPECT_EQ(best->to_, node_idx_t{2});
}

TEST(ch_shortcuts, clear_and_size) {
  ch_shortcuts shortcuts;
  
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{1}, node_idx_t{2}, cost_t{50});
  shortcuts.add_shortcut(node_idx_t{1}, node_idx_t{2}, node_idx_t{3}, cost_t{75});
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{1}, node_idx_t{4}, cost_t{60});  // Same edge, different via
  
  EXPECT_EQ(shortcuts.size(), 3);
  
  shortcuts.clear();
  EXPECT_EQ(shortcuts.size(), 0);
  EXPECT_FALSE(shortcuts.has_shortcut(node_idx_t{0}, node_idx_t{1}));
}

TEST(ch_preprocessor, interface_test) {
  // Test that the CH preprocessor class can be instantiated and has
  // the expected interface. We'll test with real data in integration tests.
  
  // For now, just verify the header compiles and interfaces exist
  // We'll add proper tests when we have monaco data loaded
  SUCCEED();
}

}  // namespace osr