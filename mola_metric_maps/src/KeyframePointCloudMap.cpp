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
#include <mrpt/system/string_utils.h>  // unitsFormat()

#include <Eigen/Dense>
#include <numeric>  // std::accumulate

//
#include <iostream>  // DEBUG, remove!

#if defined(MOLA_METRIC_MAPS_USE_TBB)
#include <tbb/parallel_for.h>
#endif

using namespace mola;

// #define DO_PROFILE_COVS 1

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

  MRPT_TODO("Implement loading of TCreationOptions into KeyframePointCloudMap::TMapDefinition::")

  if (s.sectionExists(sectionPrefix + "_insertOpts"s))
  {
    insertionOpts.loadFromConfigFile(s, sectionPrefix + "_insertOpts"s);
  }

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
  out.WriteAs<uint32_t>(keyframes_.size());
  for (const auto& [kf_id, kf] : keyframes_)
  {
    out << kf_id;
    out << kf.timestamp;
    out << kf.pose();
    if (kf.pointcloud())
    {
      out.WriteAs<uint8_t>(1);  // has point cloud
      out << *kf.pointcloud();
    }
    else
    {
      out.WriteAs<uint8_t>(0);  // no point cloud
    }
  }
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
      uint32_t n_kfs = in.ReadAs<uint32_t>();
      for (uint32_t i = 0; i < n_kfs; i++)
      {
        uint64_t kf_id;
        in >> kf_id;

        KeyFrame& kf = keyframes_[kf_id];

        in >> kf.timestamp;
        mrpt::poses::CPose3D pose;
        in >> pose;
        kf.pose(pose);
        const auto has_pointcloud = in.ReadAs<uint8_t>();
        if (has_pointcloud)
        {
          auto obj = in.ReadObject();
          auto pc  = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obj);
          ASSERT_(pc);
          kf.pointcloud(pc);
        }
      }
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
  if (cached_.boundingBox)
  {
    return *cached_.boundingBox;
  }

  // Pessimistic bounding box:
  // TODO(jlbc): To be refined once mrpt3 implements Oriented Bounding Boxes
  cached_.boundingBox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
  for (const auto& [kf_id, kf] : keyframes_)
  {
    cached_.boundingBox = cached_.boundingBox->unionWith(kf.localBoundingBox().compose(kf.pose()));
  }

  return *cached_.boundingBox;
}

bool KeyframePointCloudMap::nn_single_search(
    const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  ASSERT_(cached_.icp_search_submap);
  return cached_.icp_search_submap->pointcloud()->nn_single_search(
      query, result, out_dist_sqr, resultIndexOrID);
}

bool KeyframePointCloudMap::nn_single_search(
    const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
    uint64_t& resultIndexOrID) const
{
  ASSERT_(cached_.icp_search_submap);
  return cached_.icp_search_submap->pointcloud()->nn_single_search(
      query, result, out_dist_sqr, resultIndexOrID);
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint3Df& query, const size_t N, std::vector<mrpt::math::TPoint3Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_multiple_search(
      query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

void KeyframePointCloudMap::nn_multiple_search(
    const mrpt::math::TPoint2Df& query, const size_t N, std::vector<mrpt::math::TPoint2Df>& results,
    std::vector<float>& out_dists_sqr, std::vector<uint64_t>& resultIndicesOrIDs) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_multiple_search(
      query, N, results, out_dists_sqr, resultIndicesOrIDs);
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_radius_search(
      query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
}

void KeyframePointCloudMap::nn_radius_search(
    const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
    std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
    std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const
{
  ASSERT_(cached_.icp_search_submap);
  cached_.icp_search_submap->pointcloud()->nn_radius_search(
      query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
}

void KeyframePointCloudMap::icp_get_prepared_as_global(
    const mrpt::poses::CPose3D&                                      icp_ref_point,
    [[maybe_unused]] const std::optional<mrpt::math::TBoundingBoxf>& local_map_roi) const
{
  std::set<KeyFrameID> kfs_to_search_limited;

  //  max_search_keyframes
  // First, build a list of candidate key-frames to search into:
  std::map<double, KeyFrameID> kfs_to_search;  // key: distance to KF center
  for (auto& [kf_id, kf] : keyframes_)
  {
    if (!kf.pointcloud())
    {
      continue;
    }

    // convert query to local coordinates of the keyframe:
    const auto query_local    = kf.pose().inverseComposePoint(icp_ref_point.translation());
    const auto dist_to_kf     = query_local.norm();
    kfs_to_search[dist_to_kf] = kf_id;
  }

  MRPT_TODO("Change criteria here so more distant frames are used too");

  for (const auto& [dist, kf_id] : kfs_to_search)
  {
    kfs_to_search_limited.insert(kf_id);
    if (kfs_to_search_limited.size() >= creationOptions.max_search_keyframes)
    {
      break;
    }
  }

  // For selected KFs, build the submap, if it's different from the previous one:
  if (cached_.icp_search_kfs && *cached_.icp_search_kfs == kfs_to_search_limited)
  {
    // We are already up to date.
    return;
  }

  cached_.icp_search_kfs = kfs_to_search_limited;

  // Rebuild "cached merged KF":

  cached_.icp_search_submap.reset();
  cached_.icp_search_submap.emplace();

  for (const auto kf_id : kfs_to_search_limited)
  {
    const auto& kf = keyframes_.at(kf_id);

    if (!kf.pointcloud())
    {
      continue;  // Should never happen!
    }

    if (!cached_.icp_search_submap->pointcloud())
    {
      cached_.icp_search_submap->pointcloud(mrpt::maps::CSimplePointsMap::Create());
    }

    MRPT_TODO("Use cached global pointcloud inside KF");
    cached_.icp_search_submap->pointcloud()->insertAnotherMap(kf.pointcloud().get(), kf.pose());
  }

  // This builds the KD-tree and covariances
  cached_.icp_search_submap->buildCache();
}

void KeyframePointCloudMap::icp_cleanup() const
{
  // Do NOT free the map, we might reuse it for next ICP call.
}

void KeyframePointCloudMap::nn_search_cov2cov(
    const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
    const float max_search_distance, mp2p_icp::MatchedPointWithCovList& outPairings) const
{
  ASSERTMSG_(
      cached_.icp_search_submap,
      "Using this method requires calling icp_get_prepared_as_global() first");

  // Enforce local map to recompute its covariances to the new pose:
  const auto* localMapKF = dynamic_cast<const KeyframePointCloudMap*>(&localMap);
  ASSERTMSG_(
      localMapKF, "Implementation expects local map to be also of type KeyframePointCloudMap");

  ASSERT_EQUAL_(localMapKF->keyframes_.size(), 1U);
  auto& localKf = const_cast<KeyFrame&>(localMapKF->keyframes_.at(0));
  localKf.pose(localMapPose);

  const auto& localKfCov  = localKf.covariancesGlobal();
  const auto& localPoints = localKf.pointcloud_global();

  const auto& globalKfCov  = cached_.icp_search_submap->covariancesGlobal();
  const auto& globalPoints = cached_.icp_search_submap->pointcloud_global();

  const auto localPointCount = localPoints->size();

  const float max_sqr_dist = mrpt::square(max_search_distance);

  const auto& xs = localPoints->getPointsBufferRef_x();
  const auto& ys = localPoints->getPointsBufferRef_y();
  const auto& zs = localPoints->getPointsBufferRef_z();

  const auto& g_xs = globalPoints->getPointsBufferRef_x();
  const auto& g_ys = globalPoints->getPointsBufferRef_y();
  const auto& g_zs = globalPoints->getPointsBufferRef_z();

  globalPoints->kdTreeEnsureIndexBuilt3D();

#if defined(MOLA_METRIC_MAPS_USE_TBB)
  tbb::parallel_for(
      static_cast<size_t>(0), localPointCount,
      [&](size_t local_idx)
#else
  for (size_t local_idx = 0; local_idx < localPointCount; local_idx++)
#endif
      {
        float nn_dist_sqr = std::numeric_limits<float>::max();

        const auto nn_global_idx = globalPoints->kdTreeClosestPoint3D(
            xs[local_idx], ys[local_idx], zs[local_idx], nn_dist_sqr);

        if (nn_dist_sqr <= max_sqr_dist)
        {
          // Add pairing:
          auto& p      = outPairings.emplace_back();
          p.global_idx = nn_global_idx;
          p.local_idx  = local_idx;
          p.local      = {xs[local_idx], ys[local_idx], zs[local_idx]};
          p.global     = {g_xs[nn_global_idx], g_ys[nn_global_idx], g_zs[nn_global_idx]};

          /* Following GICP \cite segal2009gicp this should be:
           *  `(COV_{global} + R*COV_{local}*R^T)^{-1}`
           *  But localKfCov already incorporate R*C*R^T from localKf.pose(p)
           */
          p.cov_inv = (globalKfCov.at(nn_global_idx) + localKfCov.at(local_idx)).inverse_LLt();

          std::cout << p.asString();
        }
      }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
  );
#endif
}

std::string KeyframePointCloudMap::asString() const
{
  // Returns a short description of the map:
  std::ostringstream o;
  std::size_t        total_points = 0;
  for (const auto& [kf_id, kf] : keyframes_)
  {
    total_points += kf.pointcloud() ? kf.pointcloud()->size() : 0;
  }

  o << "KeyframePointCloudMap: " << keyframes_.size() << " keyframes, "
    << mrpt::system::unitsFormat(static_cast<double>(total_points)) << " points.";
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
  for (const auto& [kf_id, kf] : keyframes_)
  {
    auto obj = mrpt::opengl::CPointCloudColoured::Create();

    obj->loadFromPointsMap(kf.pointcloud().get());

    obj->setPose(kf.pose());

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
  keyframes_.clear();
  cached_.reset();
  cachedPoints_.reset();
}

bool KeyframePointCloudMap::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  // Get robot pose for insertion pose:
  mrpt::poses::CPose3D pc_in_map;
  if (robotPose)
  {
    pc_in_map = *robotPose;
  }

  // Remove old key-frames if requested:
  if (insertionOptions.remove_frames_farther_than > 0)
  {
    for (auto it = keyframes_.begin(); it != keyframes_.end();)
    {
      const double dist = pc_in_map.distanceTo(it->second.pose());
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

  // Observation must be a point cloud:
  if (auto obsPC = dynamic_cast<const mrpt::obs::CObservationPointCloud*>(&obs); obsPC)
  {
    ASSERT_(obsPC->pointcloud);

    // Add KF:
    auto& new_kf     = keyframes_[nextFreeKeyFrameID()];
    new_kf.timestamp = obs.timestamp;
    new_kf.pose(pc_in_map);
    new_kf.pointcloud(obsPC->pointcloud);

    new_kf.buildCache();
    cached_.reset();

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

bool KeyframePointCloudMap::internal_canComputeObservationLikelihood(
    const mrpt::obs::CObservation& obs) const
{
  // TODO
  return false;
}

//  =========== KeyFrame ============

void KeyframePointCloudMap::KeyFrame::updateBBox() const
{
  if (!pointcloud_)
  {
    cached_bbox_local_ = mrpt::math::TBoundingBoxf({0, 0, 0}, {0, 0, 0});
  }
  else
  {
    cached_bbox_local_ = pointcloud_->boundingBox();
  }
}

mrpt::math::TBoundingBoxf KeyframePointCloudMap::KeyFrame::localBoundingBox() const
{
  if (!cached_bbox_local_)
  {
    updateBBox();
  }
  return *cached_bbox_local_;
}

void KeyframePointCloudMap::KeyFrame::buildCache() const
{
  // Compute bbox:
  updateBBox();

  // Build KD-tree:
  ASSERT_(pointcloud_);
  pointcloud_->kdTreeEnsureIndexBuilt3D();

  // Build per-point covariances:
  computeCovariancesAndDensity();
}

void KeyframePointCloudMap::KeyFrame::updatePointsGlobal() const
{
  if (!pointcloud_)
  {
    return;
  }
  if (!pointcloud_global_)
  {
    pointcloud_global_ = mrpt::maps::CSimplePointsMap::Create();
  }
  pointcloud_global_->clear();
  pointcloud_global_->insertAnotherMap(pointcloud_.get(), pose());
}

void KeyframePointCloudMap::KeyFrame::computeCovariancesAndDensity() const
{
  ASSERT_(pointcloud_);
  const auto point_count = pointcloud_->size();

  if (cached_cov_local_.size() == point_count)
  {
    return;  // Already computed
  }

#if DO_PROFILE_COVS
  auto start = std::chrono::high_resolution_clock::now();
#endif

  // Resize:
  cached_cov_local_.resize(point_count);
  cached_cov_global_.clear();  // invalidate

  if (point_count < 3)
  {
    // Nothing to do
    cloud_density_ = 0;
    return;
  }

  // Compute using KD-tree:
  float sum_k_sq_distances = 0.0;

  const size_t K_CORRESPONDENCES = 20;
  const auto   normalization     = ((K_CORRESPONDENCES - 1) * (2 + K_CORRESPONDENCES)) / 2;

  const auto& xs = pointcloud_->getPointsBufferRef_x();
  const auto& ys = pointcloud_->getPointsBufferRef_y();
  const auto& zs = pointcloud_->getPointsBufferRef_z();

#if defined(MOLA_METRIC_MAPS_USE_TBB)
  tbb::parallel_for(
      static_cast<size_t>(0), point_count,
      [&](size_t i)
#else
  for (size_t i = 0; i < point_count; i++)
#endif
      {
        std::vector<size_t> k_indices;
        std::vector<float>  k_sq_distances;

        pointcloud_->kdTreeNClosestPoint3DIdx(
            xs[i], ys[i], zs[i], K_CORRESPONDENCES, k_indices, k_sq_distances);

        sum_k_sq_distances +=
            std::accumulate(k_sq_distances.begin() + 1, k_sq_distances.end(), 0.0f) / normalization;

        Eigen::Matrix<double, 3, -1> neighbors(3, K_CORRESPONDENCES);
        for (size_t j = 0; j < k_indices.size(); j++)
        {
          neighbors(0, static_cast<Eigen::Index>(j)) = static_cast<double>(xs[k_indices[j]]);
          neighbors(1, static_cast<Eigen::Index>(j)) = static_cast<double>(ys[k_indices[j]]);
          neighbors(2, static_cast<Eigen::Index>(j)) = static_cast<double>(zs[k_indices[j]]);
        }

        neighbors.colwise() -= neighbors.rowwise().mean().eval();
        const Eigen::Matrix3d cov = neighbors * neighbors.transpose() / K_CORRESPONDENCES;

        // Plane regularization (see DLIO'2023 or Thrun's GICP paper)
        // ------------------------------------------------------------
        // Regularization of singular values.
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // SVD sorts eigenvalues in decreasing order, so the last one
        // is the smallest (normal direction of a plane):
        const Eigen::Vector3d values = Eigen::Vector3d(1.0, 1.0, 1e-3);

        cached_cov_local_[i] = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
      }
#if defined(MOLA_METRIC_MAPS_USE_TBB)
  );
#endif

  cloud_density_ = std::sqrt(sum_k_sq_distances / static_cast<float>(point_count));

  // done.
#if DO_PROFILE_COVS
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start);
  std::cout << "Compute covs: N=" << point_count << " in "
            << static_cast<double>(duration.count()) * 1e-3 << " ms d=" << *cloud_density_ << "\n";
#endif
}

void KeyframePointCloudMap::KeyFrame::updateCovariancesGlobal() const
{
  if (cached_cov_global_.size() == cached_cov_local_.size())
  {
    return;  // Already computed
  }

  ASSERT_EQUAL_(cached_cov_local_.size(), pointcloud_->size());

  cached_cov_global_.resize(cached_cov_local_.size());

  const Eigen::Matrix3f R = pose_.getRotationMatrix().cast_float().asEigen();

  for (size_t i = 0; i < cached_cov_local_.size(); i++)
  {
    cached_cov_global_[i] = R * cached_cov_local_[i].asEigen() * R.transpose();
  }
}
