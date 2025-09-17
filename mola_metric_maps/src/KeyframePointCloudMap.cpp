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
 * @file   KeyframePointCloudMap.cpp
 * @brief  Key-frames, each keeping point cloud layers with their own KD-tree
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */

#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/CArchive.h>  // serialization

using namespace mola;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
    "mola::KeyframePointCloudMap,KeyframePointCloudMap", mola::KeyframePointCloudMap)

KeyframePointCloudMap::TMapDefinition::TMapDefinition() = default;
void KeyframePointCloudMap::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& s, const std::string& sectionPrefix)
{
  using namespace std::string_literals;

  // [<sectionNamePrefix>+"_creationOpts"]
  // const std::string sSectCreation = sectionPrefix + "_creationOpts"s;
  // MRPT_LOAD_CONFIG_VAR(voxel_size, float, s, sSectCreation);

  ASSERT_(s.sectionExists(sectionPrefix + "_insertOpts"s));
  insertionOpts.loadFromConfigFile(s, sectionPrefix + "_insertOpts"s);

  if (s.sectionExists(sectionPrefix + "_likelihoodOpts"s))
  {
    likelihoodOpts.loadFromConfigFile(s, sectionPrefix + "_likelihoodOpts"s);
  }

  if (s.sectionExists(sectionPrefix + "_renderOpts"s))
  {
    renderOpts.loadFromConfigFile(s, sectionPrefix + "_renderOpts"s);
  }
}

void KeyframePointCloudMap::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  // LOADABLEOPTS_DUMP_VAR(voxel_size, float);

  insertionOpts.dumpToTextStream(out);
  likelihoodOpts.dumpToTextStream(out);
  renderOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr KeyframePointCloudMap::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const auto* def = dynamic_cast<const KeyframePointCloudMap::TMapDefinition*>(&_def);
  ASSERT_(def);
  auto obj = KeyframePointCloudMap::Create();

  obj->insertionOptions  = def->insertionOpts;
  obj->likelihoodOptions = def->likelihoodOpts;
  obj->renderOptions     = def->renderOpts;

  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(KeyframePointCloudMap, CMetricMap, mola)

// =====================================
// Serialization
// =====================================

uint8_t KeyframePointCloudMap::serializeGetVersion() const { return 0; }
void    KeyframePointCloudMap::serializeTo(mrpt::serialization::CArchive& out) const
{
  // params:
  // out << params_;
  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
  renderOptions.writeToStream(out);

  // data:
#if 0
  out.WriteAs<uint32_t>(voxels_.size());
  for (const auto& [idx, voxel] : voxels_)
  {
  }
#endif
}

void KeyframePointCloudMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  // clear contents
  this->clear();

  switch (version)
  {
    case 0:
    {
      // params:
      // in >> params_;
      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
      renderOptions.readFromStream(in);

      // data:
#if 0
      const auto nGrids = in.ReadAs<uint32_t>();
      for (uint32_t i = 0; i < nGrids; i++)
      {
      }
#endif
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  // cache reset:
  cached_.reset();
}

///  === KeyframePointCloudMap ===

KeyframePointCloudMap::~KeyframePointCloudMap() = default;

mrpt::math::TBoundingBoxf KeyframePointCloudMap::boundingBox() const
{
  // TODO: implement real bounding box computation
  return mrpt::math::TBoundingBoxf::PlusMinusInfinity();
}

bool KeyframePointCloudMap::nn_single_search(
    const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  // TODO
  return false;
}

bool KeyframePointCloudMap::nn_single_search(
    const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  // TODO
  return false;
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint3Df& query, const size_t N, std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  // TODO
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint2Df& query, const size_t N, std::vector<mrpt::math::TPoint2Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  // TODO
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  // TODO
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  // TODO
}

std::string KeyframePointCloudMap::asString() const
{
  // Returns a short description of the map with the name of keyframes:
  std::ostringstream o;
  o << "KeyframePointCloudMap with " << keyframes_.size() << " keyframes.\n";
  return o.str();
}

void KeyframePointCloudMap::getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const
{
  MRPT_START
  if (!genericMapParams.enableSaveAs3DObject)
  {
    return;
  }

  const uint8_t alpha_u8 = mrpt::f2u8(renderOptions.color.A);

  // Create one visualization object per KF:
  for (const auto& kf : keyframes_)
  {
    auto obj = mrpt::opengl::CPointCloudColoured::Create();

    obj->loadFromPointsMap(kf.pointcloud.get());

    obj->setPose(kf.pose);

    obj->setPointSize(renderOptions.point_size);
    if (renderOptions.color.A != 1.0f)
    {
      obj->setAllPointsAlpha(alpha_u8);
    }

#if 0
    const int idx = renderOptions.recolorizeByCoordinateIndex;
    ASSERT_(idx >= 0 && idx < 3);
    float            min = .0, max = 1.f;
    constexpr double confidenceInterval = 0.02;
    obj->recolorizeByCoordinate(
        min, max, renderOptions.recolorizeByCoordinateIndex, renderOptions.colormap);
#endif

    outObj.insert(obj);
  }

  MRPT_END
}

bool KeyframePointCloudMap::isEmpty() const { return keyframes_.empty(); }

void KeyframePointCloudMap::saveMetricMapRepresentationToFile(
    const std::string& filNamePrefix) const
{
  // TODO
}

const mrpt::maps::CSimplePointsMap* KeyframePointCloudMap::getAsSimplePointsMap() const
{
  // TODO: return cachedPoints_ or recompute it
  return cachedPoints_.get();
}

// ==========================
//   Options
// ==========================

void KeyframePointCloudMap::TInsertionOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(remove_frames_farther_than, double, c, s);
}

void KeyframePointCloudMap::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
  LOADABLEOPTS_DUMP_VAR(remove_frames_farther_than, double);
}

void KeyframePointCloudMap::TInsertionOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  const int8_t version = 0;
  out << version;
  out << remove_frames_farther_than;
}

void KeyframePointCloudMap::TInsertionOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  int8_t version;
  in >> version;
  switch (version)
  {
    case 0:
    {
      in >> remove_frames_farther_than;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void KeyframePointCloudMap::TLikelihoodOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& source, const std::string& section)
{
  // TODO
}

void KeyframePointCloudMap::TLikelihoodOptions::dumpToTextStream(std::ostream& out) const
{
  // TODO
}

void KeyframePointCloudMap::TLikelihoodOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  // TODO
}

void KeyframePointCloudMap::TLikelihoodOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  // TODO
}

void KeyframePointCloudMap::TRenderOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& source, const std::string& section)
{
  // TODO
}

void KeyframePointCloudMap::TRenderOptions::dumpToTextStream(std::ostream& out) const
{
  // TODO
}

void KeyframePointCloudMap::TRenderOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  // TODO
}

void KeyframePointCloudMap::TRenderOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  // TODO
}

// ==========================
//   Protected / Private
// ==========================

void KeyframePointCloudMap::internal_clear()
{
  // TODO
  cached_.reset();
  cachedPoints_.reset();
}

bool KeyframePointCloudMap::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  mrpt::poses::CPose3D pc_in_map;
  if (robotPose)
  {
    pc_in_map = *robotPose;
  }

  if (insertionOptions.remove_frames_farther_than > 0)
  {
    for (auto it = keyframes_.begin(); it != keyframes_.end();)
    {
      const double dist = pc_in_map.distanceTo(it->pose);
      if (dist > insertionOptions.remove_frames_farther_than)
      {
        it = keyframes_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  if (auto obsPC = dynamic_cast<const mrpt::obs::CObservationPointCloud*>(&obs); obsPC)
  {
    ASSERT_(obsPC->pointcloud);
    // Add KF:
    auto& new_kf      = keyframes_.emplace_back();
    new_kf.timestamp  = obs.timestamp;
    new_kf.pose       = pc_in_map;
    new_kf.pointcloud = obsPC->pointcloud;
    return true;
  }

  // Not of supported type, we cannot insert into our map:
  return false;
}

double KeyframePointCloudMap::internal_computeObservationLikelihood(
    const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const
{
  // TODO
  return .0;
}

double KeyframePointCloudMap::internal_computeObservationLikelihoodPointCloud3D(
    const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
    const std::size_t num_pts) const
{
  // TODO
  return .0;
}

void KeyframePointCloudMap::internal_insertPointCloud3D(
    const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
    const std::size_t num_pts)
{
  // TODO
}

bool KeyframePointCloudMap::internal_canComputeObservationLikelihood(
    const mrpt::obs::CObservation& obs) const
{
  // TODO
  return false;
}

//  =========== KeyFrame ============

mrpt::math::TBoundingBoxf KeyframePointCloudMap::KeyFrame::localBoundingBox() const
{
  if (cached_bbox_)
  {
    return *cached_bbox_;
  }
  if (!pointcloud)
  {
    return mrpt::math::TBoundingBoxf({0, 0, 0}, {0, 0, 0});
  }
  cached_bbox_ = pointcloud->boundingBox();
  return *cached_bbox_;
}
