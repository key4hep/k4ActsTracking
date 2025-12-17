/*
 * Copyright (c) 2014-2024 Key4hep-Project.
 *
 * This file is part of Key4hep.
 * See https://key4hep.github.io/key4hep-doc/ for further info.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include "Acts/Definitions/TrackParametrization.hpp"
#include "Acts/EventData/SourceLink.hpp"
#include "Acts/EventData/Types.hpp"
#include "Acts/EventData/detail/ParameterTraits.hpp"
#include "Acts/EventData/detail/PrintParameters.hpp"

#include <boost/container/static_vector.hpp>

#include <array>
#include <variant>

namespace ACTSTracking {

  template <typename indices_t> class VariableSizeMeasurement {
  public:
    static constexpr std::size_t kFullSize = Acts::detail::kParametersSize<indices_t>;

    using Scalar = double;

    using SubspaceIndex   = std::uint8_t;
    using SubspaceIndices = boost::container::static_vector<SubspaceIndex, kFullSize>;

    /// Vector type containing for measured parameter values.
    template <std::size_t dim> using ParametersVector         = Eigen::Matrix<Scalar, dim, 1>;
    template <std::size_t dim> using ParametersVectorMap      = Eigen::Map<ParametersVector<dim>>;
    template <std::size_t dim> using ConstParametersVectorMap = Eigen::Map<const ParametersVector<dim>>;
    using EffectiveParametersVector                           = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using EffectiveParametersVectorMap                        = Eigen::Map<EffectiveParametersVector>;
    using ConstEffectiveParametersVectorMap                   = Eigen::Map<const EffectiveParametersVector>;
    template <std::size_t dim> using CovarianceMatrix         = Eigen::Matrix<Scalar, dim, dim>;
    template <std::size_t dim> using CovarianceMatrixMap      = Eigen::Map<CovarianceMatrix<dim>>;
    template <std::size_t dim> using ConstCovarianceMatrixMap = Eigen::Map<const CovarianceMatrix<dim>>;
    using EffectiveCovarianceMatrix                           = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using EffectiveCovarianceMatrixMap                        = Eigen::Map<EffectiveCovarianceMatrix>;
    using ConstEffectiveCovarianceMatrixMap                   = Eigen::Map<const EffectiveCovarianceMatrix>;

    using FullParametersVector = Acts::ActsVector<kFullSize>;
    using FullCovarianceMatrix = Acts::ActsSquareMatrix<kFullSize>;

    using ProjectionMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, kFullSize>;
    using ExpansionMatrix  = Eigen::Matrix<Scalar, kFullSize, Eigen::Dynamic>;

    template <typename other_indices_t, std::size_t kSize, typename parameters_t, typename covariance_t>
    VariableSizeMeasurement(Acts::SourceLink source, const std::array<other_indices_t, kSize>& subspaceIndices,
                            const Eigen::MatrixBase<parameters_t>& params, const Eigen::MatrixBase<covariance_t>& cov)
        : m_source(std::move(source)) {
      static_assert(kSize == parameters_t::RowsAtCompileTime, "Parameter size mismatch");
      static_assert(kSize == covariance_t::RowsAtCompileTime, "Covariance rows mismatch");
      static_assert(kSize == covariance_t::ColsAtCompileTime, "Covariance cols mismatch");

      m_subspaceIndices.resize(subspaceIndices.size());
      std::transform(subspaceIndices.begin(), subspaceIndices.end(), m_subspaceIndices.begin(),
                     [](auto index) { return static_cast<SubspaceIndex>(index); });

      parameters<kSize>() = params;
      covariance<kSize>() = cov;
    }

    VariableSizeMeasurement()                                          = delete;
    VariableSizeMeasurement(const VariableSizeMeasurement&)            = default;
    VariableSizeMeasurement(VariableSizeMeasurement&&)                 = default;
    ~VariableSizeMeasurement()                                         = default;
    VariableSizeMeasurement& operator=(const VariableSizeMeasurement&) = default;
    VariableSizeMeasurement& operator=(VariableSizeMeasurement&&)      = default;

    const Acts::SourceLink& sourceLink() const { return m_source; }

    constexpr std::size_t size() const { return m_subspaceIndices.size(); }

    bool contains(indices_t i) const {
      return std::find(m_subspaceIndices.begin(), m_subspaceIndices.end(), i) != m_subspaceIndices.end();
    }

    std::size_t indexOf(indices_t i) const {
      auto it = std::find(m_subspaceIndices.begin(), m_subspaceIndices.end(), i);
      assert(it != m_subspaceIndices.end());
      return std::distance(m_subspaceIndices.begin(), it);
    }

    const SubspaceIndices& subspaceIndices() const { return m_subspaceIndices; }

    template <std::size_t dim> Acts::SubspaceIndices<dim> subspaceIndices() const {
      assert(dim == size());
      Acts::SubspaceIndices<dim> result;
      std::copy(m_subspaceIndices.begin(), m_subspaceIndices.end(), result.begin());
      return result;
    }

    Acts::BoundSubspaceIndices boundSubsetIndices() const {
      Acts::BoundSubspaceIndices result = Acts::kBoundSubspaceIndicesInvalid;
      std::copy(m_subspaceIndices.begin(), m_subspaceIndices.end(), result.begin());
      return result;
    }

    template <std::size_t dim> ConstParametersVectorMap<dim> parameters() const {
      assert(dim == size());
      return ConstParametersVectorMap<dim>{m_params.data()};
    }

    template <std::size_t dim> ParametersVectorMap<dim> parameters() {
      assert(dim == size());
      return ParametersVectorMap<dim>{m_params.data()};
    }

    ConstEffectiveParametersVectorMap parameters() const {
      return ConstEffectiveParametersVectorMap{m_params.data(), static_cast<Eigen::Index>(size())};
    }

    EffectiveParametersVectorMap parameters() {
      return EffectiveParametersVectorMap{m_params.data(), static_cast<Eigen::Index>(size())};
    }

    template <std::size_t dim> ConstCovarianceMatrixMap<dim> covariance() const {
      assert(dim == size());
      return ConstCovarianceMatrixMap<dim>{m_cov.data()};
    }

    template <std::size_t dim> CovarianceMatrixMap<dim> covariance() {
      assert(dim == size());
      return CovarianceMatrixMap<dim>{m_cov.data()};
    }

    ConstEffectiveCovarianceMatrixMap covariance() const {
      return ConstEffectiveCovarianceMatrixMap{m_cov.data(), static_cast<Eigen::Index>(size()),
                                               static_cast<Eigen::Index>(size())};
    }

    EffectiveCovarianceMatrixMap covariance() {
      return EffectiveCovarianceMatrixMap{m_cov.data(), static_cast<Eigen::Index>(size()),
                                          static_cast<Eigen::Index>(size())};
    }

    FullParametersVector fullParameters() const {
      FullParametersVector result = FullParametersVector::Zero();
      for (std::size_t i = 0; i < size(); ++i) {
        result[m_subspaceIndices[i]] = parameters()[i];
      }
      return result;
    }

    FullCovarianceMatrix fullCovariance() const {
      FullCovarianceMatrix result = FullCovarianceMatrix::Zero();
      for (std::size_t i = 0; i < size(); ++i) {
        for (std::size_t j = 0; j < size(); ++j) {
          result(m_subspaceIndices[i], m_subspaceIndices[j]) = covariance()(i, j);
        }
      }
      return result;
    }

  private:
    Acts::SourceLink                          m_source;
    SubspaceIndices                           m_subspaceIndices;
    std::array<Scalar, kFullSize>             m_params{};
    std::array<Scalar, kFullSize * kFullSize> m_cov{};
  };

  using BoundVariableMeasurement = VariableSizeMeasurement<Acts::BoundIndices>;

  using Measurement = BoundVariableMeasurement;

  using MeasurementContainer = std::vector<Measurement>;

  template <typename parameters_t, typename covariance_t, typename indices_t, typename... tail_indices_t>
  VariableSizeMeasurement<indices_t> makeMeasurement(Acts::SourceLink                       source,
                                                     const Eigen::MatrixBase<parameters_t>& params,
                                                     const Eigen::MatrixBase<covariance_t>& cov, indices_t index0,
                                                     tail_indices_t... tailIndices) {
    using IndexContainer = std::array<indices_t, 1u + sizeof...(tail_indices_t)>;
    return {std::move(source), IndexContainer{index0, tailIndices...}, params, cov};
  }

}  // namespace ACTSTracking
