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
 * @file   KeyframePointCloudMap.h
 * @brief  Key-frames, each keeping point cloud layers with their own KD-tree
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */
#pragma once

#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/math/TBoundingBox.h>

#include <deque>
#include <optional>

namespace mola
{
/** An efficient storage class for large point clouds built as keyframes, each having an associated
 * local cloud.
 *
 * The user of the class is responsible for processing raw observations into
 * mrpt::obs::CObservationPointCloud observations, the only ones allowed as input to the insert*()
 * methods, with points already transformed from the sensor frame to the vehicle (`base_link`)
 * frame. This can be easily done with mp2p_icp::Generator, plus an optional filtering pipeline.
 *
 * Each key-frame is responsible of keeping its own KD-tree for NN searches and keeping up-to-date
 * covariances for each point local vicinity.
 */
class KeyframePointCloudMap : public mrpt::maps::CMetricMap,
                              public mrpt::maps::NearestNeighborsCapable
{
  DEFINE_SERIALIZABLE(KeyframePointCloudMap, mola)
 public:
  // Prevent copying and moving
  KeyframePointCloudMap(const KeyframePointCloudMap&)            = default;
  KeyframePointCloudMap& operator=(const KeyframePointCloudMap&) = default;
  KeyframePointCloudMap(KeyframePointCloudMap&&)                 = default;
  KeyframePointCloudMap& operator=(KeyframePointCloudMap&&)      = default;

  /** @name Basic API for construction and main parameters
   *  @{ */

  /**
   * @brief Constructor / default ctor
   */
  KeyframePointCloudMap() = default;

  ~KeyframePointCloudMap();

  /** @} */

  /** @name Data structures
   *  @{ */

  /** @} */

  /** @name Data access API
   *  @{ */
  // clear(): available in base class

  /** Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if
   * there are no points. Results are cached unless the map is somehow
   * modified to avoid repeated calculations.
   */
  mrpt::math::TBoundingBoxf boundingBox() const override;

  // void visitAllKeyFrames(const std::function<void(const global_index3d_t&, const VoxelData&)>& f)
  // const;

  /** @} */

  /** @name API of the NearestNeighborsCapable virtual interface
  @{ */
  [[nodiscard]] bool nn_has_indices_or_ids() const override
  {
    return false;  // No, they are not contiguous indices, but "IDs"
  }
  [[nodiscard]] size_t nn_index_count() const override
  {
    return 0;  // Ignored when nn_has_indices_or_ids() => false
  }

  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result, float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override;
  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result, float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint3Df& query, const size_t N,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_multiple_search(
      const mrpt::math::TPoint2Df& query, const size_t N,
      std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override;
  void nn_radius_search(
      const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;
  void nn_radius_search(
      const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
      std::vector<mrpt::math::TPoint2Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs, size_t maxPoints) const override;

  template <size_t MAX_KNN>
  void nn_multiple_search_impl(
      const mrpt::math::TPoint3Df& query, const size_t N,
      std::vector<mrpt::math::TPoint3Df>& results, std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const;

  /** @} */

  /** @name Public virtual methods implementation for CMetricMap
   *  @{ */

  /** Returns a short description of the map. */
  std::string asString() const override;

  void getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const override;

  /** Returns true if the map is empty */
  bool isEmpty() const override;

  /** This virtual method saves the map to a file "filNamePrefix"+<
   * some_file_extension >, as an image or in any other applicable way (Notice
   * that other methods to save the map may be implemented in classes
   * implementing this virtual interface).  */
  void saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const override;

  /// Returns a cached point cloud view of the entire map.
  /// Not efficient at all. Only for MOLA->ROS2 bridge.
  const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const override;

  /** @} */

  /** Options for insertObservation()
   */
  struct TInsertionOptions : public mrpt::config::CLoadableOptions
  {
    TInsertionOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);
  };
  TInsertionOptions insertionOptions;

  /** Options used when evaluating "computeObservationLikelihood" in the
   * derived classes.
   * \sa CObservation::computeObservationLikelihood
   */
  struct TLikelihoodOptions : public mrpt::config::CLoadableOptions
  {
    TLikelihoodOptions() = default;

    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    void writeToStream(mrpt::serialization::CArchive& out) const;
    void readFromStream(mrpt::serialization::CArchive& in);

    /** Sigma (standard deviation, in meters) of the Gaussian observation
     *  model used to model the likelihood (default= 0.5 m) */
    double sigma_dist = 0.5;

    /** Maximum distance in meters to consider for the numerator divided by
     * "sigma_dist", so that each point has a minimum (but very small)
     * likelihood to avoid underflows (default=1.0 meters) */
    double max_corr_distance = 1.0;

    /** Speed up the likelihood computation by considering only one out of N
     * rays (default=10) */
    uint32_t decimation = 10;
  };
  TLikelihoodOptions likelihoodOptions;

  /** Rendering options, used in getAs3DObject()
   */
  struct TRenderOptions : public mrpt::config::CLoadableOptions
  {
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string&                   section) override;  // See base docs
    void dumpToTextStream(std::ostream& out) const override;  // See base docs

    /** Binary dump to stream - used in derived classes' serialization */
    void writeToStream(mrpt::serialization::CArchive& out) const;
    /** Binary dump to stream - used in derived classes' serialization */
    void readFromStream(mrpt::serialization::CArchive& in);

    float point_size = 1.0f;

    /** Color of points. Superseded by colormap if the latter is set. */
    mrpt::img::TColorf color{.0f, .0f, 1.0f};

    /** Colormap for points (index is "z" coordinates) */
    mrpt::img::TColormap colormap = mrpt::img::cmHOT;

    /** If colormap!=mrpt::img::cmNONE, use this coordinate
     *  as color index: 0=x  1=y  2=z
     */
    uint8_t recolorizeByCoordinateIndex = 2;
  };
  TRenderOptions renderOptions;

 public:
  // Interface for use within a mrpt::maps::CMultiMetricMap:
  MAP_DEFINITION_START(KeyframePointCloudMap)

  mola::KeyframePointCloudMap::TInsertionOptions  insertionOpts;
  mola::KeyframePointCloudMap::TLikelihoodOptions likelihoodOpts;
  mola::KeyframePointCloudMap::TRenderOptions     renderOpts;
  MAP_DEFINITION_END(KeyframePointCloudMap)

 private:
  /** Key-frame data */
  class KeyFrame
  {
   public:
    KeyFrame() = default;

    /**  Local metric map for this key-frame. Points are already transformed from the sensor frame
     * to the vehicle ("base_link") frame.
     */
    mrpt::maps::CPointsMap::Ptr pointcloud;
    mrpt::poses::CPose3D        pose;  //!< Pose of the key-frame in the map reference frame
    mrpt::Clock::time_point     timestamp;  //!< Timestamp of the key-frame (from observation)

    mrpt::math::TBoundingBoxf localBoundingBox() const;

    void invalidateCache() { cached_bbox_.reset(); }

   private:
    mutable std::optional<mrpt::math::TBoundingBoxf> cached_bbox_;
  };

  std::deque<KeyFrame> keyframes_;

  struct CachedData
  {
    CachedData() = default;

    void reset() { *this = CachedData(); }

    mutable std::optional<mrpt::math::TBoundingBoxf> boundingBox_;
  };

  CachedData cached_;

 protected:
  // See docs in base CMetricMap class:
  void internal_clear() override;

 private:
  // See docs in base CMetricMap class:
  bool internal_insertObservation(
      const mrpt::obs::CObservation&                   obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;
  // See docs in base class
  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const override;

  double internal_computeObservationLikelihoodPointCloud3D(
      const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
      const std::size_t num_pts) const;

  /** - (xs,ys,zs): Sensed point local coordinates in the robot frame.
   *  - pc_in_map: SE(3) pose of the robot in the map frame.
   */
  void internal_insertPointCloud3D(
      const mrpt::poses::CPose3D& pc_in_map, const float* xs, const float* ys, const float* zs,
      const std::size_t num_pts);

  // See docs in base class
  bool internal_canComputeObservationLikelihood(const mrpt::obs::CObservation& obs) const override;

  /// Used for getAsSimplePointsMap only.
  mutable mrpt::maps::CSimplePointsMap::Ptr cachedPoints_;

  /// Convert a KF index and a local point index into a global index:
  uint64_t toGlobalIndex(const size_t kf_idx, const size_t local_pt_idx) const
  {
    // Build 64 bits from 32bit kf_idx and 32bit local_pt_idx:
    return (static_cast<uint64_t>(kf_idx) << 32) | static_cast<uint64_t>(local_pt_idx);
  }
  /// Inverse of toGlobalIndex(), returning kf_idx and local_pt_idx as a tuple:
  std::tuple<size_t, size_t> fromGlobalIndex(const uint64_t global_idx) const
  {
    const size_t kf_idx       = static_cast<size_t>(global_idx >> 32);
    const size_t local_pt_idx = static_cast<size_t>(global_idx & 0xFFFFFFFF);
    return {kf_idx, local_pt_idx};
  }
};

}  // namespace mola
