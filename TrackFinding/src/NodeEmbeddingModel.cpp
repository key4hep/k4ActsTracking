#include "NodeEmbeddingModel.h"

#include <numeric>
#include <ranges>

namespace mlutils {
std::vector<float> extractHitInformation(const edm4hep::TrackerHitPlaneCollection& hits) {
  // Could use a std::array here, but that would make switching between 3D and
  // 4D a bit more cumbersome
  std::vector<std::vector<float>> embeddingInputs{};
  embeddingInputs.reserve(hits.size());

  for (const auto hit : hits) {
    std::vector<float> hitInfo = {static_cast<float>(hit.getPosition().x), static_cast<float>(hit.getPosition().y),
                                  static_cast<float>(hit.getPosition().z), hit.getTime()};
    embeddingInputs.emplace_back(std::move(hitInfo));
  }
  return mlutils::flatten(embeddingInputs);
}

} // namespace mlutils
