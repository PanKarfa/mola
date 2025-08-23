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
 * @file   factors-common.h
 * @brief  Includes all headers for common types of world-model factors
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/factors/FactorConstVelKinematics.h>
#include <mola_kernel/factors/FactorRelativePose3.h>
#include <mola_kernel/factors/FactorStereoProjectionPose.h>
#include <mola_kernel/factors/SmartFactorIMU.h>
#include <mola_kernel/factors/SmartFactorStereoProjectionPose.h>

#include <memory>

namespace mola
{
/** Placeholder for a generic factor of user-defined types */
using FactorOther = std::shared_ptr<FactorBase>;

}  // namespace mola
