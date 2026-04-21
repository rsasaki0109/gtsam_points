// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <atomic>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>

namespace gtsam_points {

class IncrementalFixedLagSmootherExtWithFallback : public IncrementalFixedLagSmootherExt {
public:
  IncrementalFixedLagSmootherExtWithFallback(double smootherLag, const ISAM2Params& parameters);
  ~IncrementalFixedLagSmootherExtWithFallback() override;

  void set_fix_variable_types(const std::vector<std::pair<char, int>>& var_types) { fix_variable_types = var_types; }

  void set_force_fallback_interval(int interval) { force_fallback_interval = interval; }

  Result update(
    const gtsam::NonlinearFactorGraph& newFactors = gtsam::NonlinearFactorGraph(),
    const gtsam::Values& newTheta = gtsam::Values(),  //
    const KeyTimestampMap& timestamps = KeyTimestampMap(),
    const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices()) override;

  bool fallbackHappened() const { return fallback_happend; }
  gtsam::Values calculateEstimate() const override;

  const gtsam::Value& calculateEstimate(gtsam::Key key) const;

  template <class VALUE>
  VALUE calculateEstimate(Key key) const {
    if (force_fallback_interval > 0 && fallback_happend.load()) {
      return values.at(key).cast<VALUE>();
    }

    try {
      const VALUE value = smoother->calculateEstimate<VALUE>(key);
      auto found = values.find(key);
      if (found != values.end()) {
        values.insert_or_assign(key, value);
      }

      return value;
    } catch (std::exception& e) {
      std::cerr << "warning: an exception was caught in fixed-lag smoother update!!" << std::endl;
      std::cerr << "       : " << e.what() << std::endl;

      fallback_smoother();
      try {
        const auto& value = smoother->calculateEstimate(key);
        auto found = values.find(key);
        if (found != values.end()) {
          const_cast<gtsam::Value&>(found->value) = value;  // Bad practice!!
        }

        return value.cast<VALUE>();
      } catch (std::exception& fallback_error) {
        std::cerr << "warning: fallback calculateEstimate still failed, using cached value!!" << std::endl;
        std::cerr << "       : " << fallback_error.what() << std::endl;

        auto found = values.find(key);
        if (found == values.end()) {
          throw;
        }

        return found->value.cast<VALUE>();
      }
    }
  }

  const NonlinearFactorGraph& getFactors() const { return smoother->getFactors(); }
  const GaussianFactorGraph& getLinearFactors() const { return smoother->getLinearFactors(); }
  const Values& getLinearizationPoint() const { return smoother->getLinearizationPoint(); }
  const VectorValues& getDelta() const { return smoother->getDelta(); }

  Matrix marginalCovariance(Key key) const { return smoother->marginalCovariance(key); }

private:
  void update_fallback_state();
  void fallback_smoother() const;

private:
  double current_stamp;
  mutable gtsam::Values values;
  mutable gtsam::Values original_values;  // insert-only snapshot of input theta (no mutation by calculateEstimate)
  mutable gtsam::NonlinearFactorGraph factors;
  mutable std::atomic_bool fallback_happend;
  std::unordered_map<gtsam::Key, std::vector<gtsam::NonlinearFactor::shared_ptr>> factor_map;
  mutable gtsam::FixedLagSmootherKeyTimestampMap stamps;

  mutable std::unique_ptr<IncrementalFixedLagSmootherExt> smoother;

  std::vector<std::pair<char, int>> fix_variable_types;  // (chr, type)  type: 0=Pose3, 1=Vector3, 2=imuBias::ConstantBias

  int force_fallback_interval = 0;  // 0 = disabled; otherwise fire fallback every N updates for determinism
  mutable int update_count = 0;
};
}  // namespace gtsam_points
