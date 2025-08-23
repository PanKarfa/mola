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
 * @file   LandmarkPoint3.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/entities/EntityBase.h>
#include <mrpt/math/TPoint3D.h>

namespace mola
{
/** A 3D point landmark.
 *
 * \ingroup mola_kernel_entities_grp
 */
class LandmarkPoint3 : public EntityBase
{
  DEFINE_SERIALIZABLE(LandmarkPoint3, mola)

 public:
  mrpt::math::TPoint3D point;
};

}  // namespace mola
