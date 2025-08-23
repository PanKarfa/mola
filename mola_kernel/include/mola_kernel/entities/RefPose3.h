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
 * @file   RefPose3.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/entities/EntityBase.h>

namespace mola
{
/** A Reference SE(3) keyframe.
 * Does not hold raw observations.
 * This kind of frame is used as "coordinate origin" for both, absolute and
 * relative maps (submaps). Global SLAM frameworks should have only one entity
 * of this class, submapping approaches will have several instances.
 *
 * \ingroup mola_kernel_entities_grp
 */
class RefPose3 : public EntityBase
{
  DEFINE_SERIALIZABLE(RefPose3, mola)

 public:
};

}  // namespace mola
