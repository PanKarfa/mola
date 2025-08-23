/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   SmartFactorIMU.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 28, 2019
 */
#pragma once

#include <mola_kernel/factors/FactorBase.h>

#include <deque>

namespace mola
{
class BackEndBase;

/** Smart factor for a set of IMU measurements (acceleration & gyroscope).
 * \ingroup mola_kernel_factors_grp
 */
class SmartFactorIMU : public FactorBase
{
  DEFINE_SERIALIZABLE(SmartFactorIMU, mola)

 public:
  SmartFactorIMU() = default;

  // TODO: Add configurable noise model & IMU pose:
  SmartFactorIMU(BackEndBase* slam_backend);

  std::size_t edge_count() const override;
  mola::id_t  edge_indices(const std::size_t i) const override;

  void integrateMeasurement(
      double accx, double accy, double accz, double wx, double wy, double wz, double dt);

  void createIMUFactor(mola::id_t prev_pose_kf, mola::id_t new_pose_kf);

  enum NewState
  {
    NONE = 0,
    MEASURE,
    FACTOR
  };

  NewState new_state_{NewState::NONE};

  double     ax_, ay_, az_, wx_, wy_, wz_, dt_;
  mola::id_t prev_pose_kf_, new_pose_kf_;

 private:
  BackEndBase* slam_backend_{nullptr};
};

}  // namespace mola
