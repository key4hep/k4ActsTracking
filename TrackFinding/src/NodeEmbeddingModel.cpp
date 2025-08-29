#include "NodeEmbeddingModel.h"

#include <numeric>
#include <ranges>

namespace mlutils {
std::vector<std::vector<float>>
NodeEmbeddingModel::extractInformation(const std::vector<const edm4hep::TrackerHitPlaneCollection*>& collections) {

  // Could use a std::array here, but that would make switching between 3D and
  // 4D a bit more cumbersome
  std::vector<std::vector<float>> embeddingInputs{};
  auto sizes = collections | std::views::transform([](const auto* c) { return c->size(); });
  auto totalHits = std::accumulate(sizes.begin(), sizes.end(), 0);
  embeddingInputs.reserve(totalHits);

  for (const auto* coll : collections) {
    for (const auto hit : *coll) {
      std::vector<float> hitInfo = {static_cast<float>(hit.getPosition().x), static_cast<float>(hit.getPosition().y),
                                    static_cast<float>(hit.getPosition().z), hit.getTime()};
      embeddingInputs.emplace_back(std::move(hitInfo));
    }
  }
  return embeddingInputs;
}

} // namespace mlutils
