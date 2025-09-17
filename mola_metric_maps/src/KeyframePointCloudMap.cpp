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
  return "KeyframePointCloudMap: <TODO description>";
}

void KeyframePointCloudMap::getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const
{
  // TODO: visualization code
}

bool KeyframePointCloudMap::isEmpty() const
{
  // TODO
  return true;
}

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
    const mrpt::config::CConfigFileBase& source, const std::string& section)
{
  // TODO
}

void KeyframePointCloudMap::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
  // TODO
}

void KeyframePointCloudMap::TInsertionOptions::writeToStream(
    mrpt::serialization::CArchive& out) const
{
  // TODO
}

void KeyframePointCloudMap::TInsertionOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  // TODO
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
  // TODO
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
  for (const auto& [name, layer] : metric_map.layers)
  {
    const auto bb = layer->boundingBox();
    if (!cached_bbox_)
    {
      cached_bbox_ = bb;
    }
    else
    {
      *cached_bbox_ = cached_bbox_->unionWith(bb);
    }
  }
  return *cached_bbox_;
}
