#include "kspwlo/impl/esx_impl.hpp"

namespace kspwlo_impl {
bool check_feasibility(const std::vector<double> &overlaps) {
  auto valid_overlapping =
      std::find_if(std::begin(overlaps), std::end(overlaps),
                   [](const auto &v) { return v > 0; });
  return !(valid_overlapping == std::end(overlaps));
}

} // namespace kspwlo_impl
