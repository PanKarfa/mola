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
 * @file   RelPose3.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/entities/EntityRelativeBase.h>
#include <mrpt/math/TPose3D.h>

namespace mola
{
/** A relative SE(3) pose (e.g. a sensor pose wrt the vehicle)
 *
 * \ingroup mola_kernel_entities_grp
 */
class RelPose3 : public EntityRelativeBase
{
  DEFINE_SERIALIZABLE(RelPose3, mola)

 public:
  /** The up-to-date value of this entity. */
  mrpt::math::TPose3D relpose_value;
};

}  // namespace mola
