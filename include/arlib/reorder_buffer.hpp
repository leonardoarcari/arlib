#ifndef ARLIB_REORDER_BUFFER_HPP
#define ARLIB_REORDER_BUFFER_HPP

#include <algorithm>
#include <arlib/details/arlib_utils.hpp>

namespace arlib {
class ReorderBuffer {
public:
  /**
   * Sorts elements in [first, last) by their length.
   * @tparam RandomIt
   * @param first
   * @param last
   */
  template <typename RandomIt>
  static void by_length(RandomIt first, RandomIt last) {
    auto cmp = [](auto const &a, auto const &b) {
      return a.length() < b.length();
    };
    std::sort(first, last, cmp);
  }

  /**
   * Sorts elements in [first, last) by minimum relative similarity, that is:
   *   - The first element is the path with minimum length
   *   - Let s be the number of sorted elements
   *   - Until @p k elements are sorted, find first element in [first+s, last)
   *     such that its similarity with all the already-sorted elements is less
   *     than @p theta. If you don't find, pick the one with minimum similarity.
   *
   * @pre Elements in [first, last) are sorted by length
   *
   * @tparam ForwardIt must meet the requirements ForwardIterator.
   * @param first iterator to the first element in the range
   * @param last iterator to the element following the last one in the range
   * @param k the number of elements to sort
   * @param theta the similarity threshold
   */
  template <typename Graph, typename ForwardIt>
  static void by_relative_similarity(Graph const &G, ForwardIt first,
                                     ForwardIt last, long k, double theta) {
    using namespace boost;
    auto weight = get(boost::edge_weight, G);
    auto count = 0;
    auto cur = first;
    while (count < k && cur != last) {
      auto found = false;
      auto sims = std::vector<std::pair<double, ForwardIt>>{};
      for (auto it = std::next(cur); it != last; ++it) {
        auto sim = details::compute_similarity(*it, *cur, weight);
        sims.emplace_back(sim, it);
        if (is_valid_candidate(first, cur, sim, theta)) {
          std::iter_swap(std::next(cur), it);
          found = true;
          break;
        }
      }
      if (!found) {
        if (!sims.empty()) {
          auto smallest = std::min_element(
              sims.cbegin(), sims.cend(),
              [](auto const &a, auto const &b) { return a.first < b.first; });
          std::iter_swap(std::next(cur), smallest->second);
        }
      }
      ++count;
      ++cur;
    }
  }

  /**
   * Invokes by_relative_similarity() with k = std::distance(first, last)
   *
   * @see by_relative_similarity(ForwardIt first,
   *                             ForwardIt last,
   *                             long k,
   *                             double theta)
   */
  template <typename ForwardIt>
  static void by_relative_similarity(ForwardIt first, ForwardIt last,
                                     double theta) {
    auto k = std::distance(first, last);
    by_relative_similarity(first, last, k, theta);
  }

private:
  template <typename ForwardIt>
  static bool is_valid_candidate(ForwardIt first, ForwardIt cur, double sim,
                                 double theta) {
    auto is_valid = true;
    for (auto jt = first; jt != std::next(cur); ++jt) {
      if (sim > theta) {
        is_valid = false;
        break;
      }
    }
    return is_valid;
  }
};
} // namespace arlib

#endif // KSPWLO_REORDER_BUFFER_HPP
