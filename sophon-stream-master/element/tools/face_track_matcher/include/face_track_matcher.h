//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#ifndef SOPHON_STREAM_ELEMENT_FACE_TRACK_MATCHER_H_
#define SOPHON_STREAM_ELEMENT_FACE_TRACK_MATCHER_H_

#include <map>
#include <mutex>
#include <unordered_map>

#include "common/object_metadata.h"
#include "element.h"

namespace sophon_stream {
namespace element {
namespace face_track_matcher {

class FaceTrackMatcher : public ::sophon_stream::framework::Element {
 public:
  FaceTrackMatcher();
  ~FaceTrackMatcher() override;

  common::ErrorCode initInternal(const std::string& json) override;
  common::ErrorCode doWork(int dataPipeId) override;

  static constexpr const char* CONFIG_INTERNAL_MAIN_PORT_FIELD = "main_port";
  static constexpr const char* CONFIG_INTERNAL_FACE_PORT_FIELD = "face_port";
  static constexpr const char* CONFIG_INTERNAL_FACE_CACHE_MAX_AGE_MS_FIELD =
      "face_cache_max_age_ms";
  static constexpr const char* CONFIG_INTERNAL_MAX_STATUS_LAG_MS_FIELD =
      "max_status_lag_ms";
  static constexpr const char* CONFIG_INTERNAL_MIN_OVERLAP_RATIO_FIELD =
      "min_face_person_overlap_ratio";

 private:
  struct MainFrameCacheEntry {
    long long timestampMs = -1;
    long long frameId = -1;
    std::shared_ptr<common::ObjectMetadata> objectMetadata;
  };

  struct PendingFaceFrameEntry {
    long long timestampMs = -1;
    long long frameId = -1;
    std::shared_ptr<common::ObjectMetadata> objectMetadata;
  };

  using ChannelMainFrameCacheMap = std::map<long long, MainFrameCacheEntry>;
  using ChannelPendingFaceFrameCacheMap =
      std::map<long long, PendingFaceFrameEntry>;

  static long long resolveTimestampMs(
      const std::shared_ptr<common::ObjectMetadata>& objectMetadata);

  void cacheMainFrame(
      const std::shared_ptr<common::ObjectMetadata>& objectMetadata);
  void cachePendingFaceFrameLocked(
      const std::shared_ptr<common::ObjectMetadata>& objectMetadata,
      long long timestampMs);
  void pruneExpiredEntriesLocked(int channelId, long long nowMs);
  const MainFrameCacheEntry* findBestMainFrameLocked(int channelId,
                                                     long long frameId,
                                                     long long timestampMs);
  const PendingFaceFrameEntry* findBestPendingFaceFrameLocked(
      int channelId, long long frameId, long long timestampMs);
  common::ErrorCode emitMatchedFaces(
      const std::shared_ptr<common::ObjectMetadata>& faceObjectMetadata,
      const MainFrameCacheEntry& mainFrameEntry);
  common::ErrorCode emitEndOfStream(
      const std::shared_ptr<common::ObjectMetadata>& mainObjectMetadata);

  int mMainPort = 0;
  int mFacePort = 1;
  int mFaceCacheMaxAgeMs = 1200;
  int mMaxStatusLagMs = 15000;
  float mMinFacePersonOverlapRatio = 0.2f;

  std::mutex mCacheMutex;
  std::unordered_map<int, ChannelMainFrameCacheMap> mMainFrameCache;
  std::unordered_map<int, ChannelPendingFaceFrameCacheMap> mPendingFaceFrameCache;
  std::unordered_map<int, long long> mSubFrameIdMap;
};

}  // namespace face_track_matcher
}  // namespace element
}  // namespace sophon_stream

#endif
