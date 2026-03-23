//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#ifndef SOPHON_STREAM_ELEMENT_TRACK_STATUS_ATTACH_H_
#define SOPHON_STREAM_ELEMENT_TRACK_STATUS_ATTACH_H_

#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/object_metadata.h"
#include "element.h"

namespace sophon_stream {
namespace element {
namespace track_status_attach {

class TrackStatusAttach : public ::sophon_stream::framework::Element {
 public:
  TrackStatusAttach();
  ~TrackStatusAttach() override;

  common::ErrorCode initInternal(const std::string& json) override;
  common::ErrorCode doWork(int dataPipeId) override;

  static constexpr const char* CONFIG_INTERNAL_MAIN_PORT_FIELD = "main_port";
  static constexpr const char* CONFIG_INTERNAL_STATUS_PORT_FIELD = "status_port";
  static constexpr const char* CONFIG_INTERNAL_TRACK_TIMEOUT_MS_FIELD =
      "track_timeout_ms";
  static constexpr const char* CONFIG_INTERNAL_MAX_STATUS_LAG_MS_FIELD =
      "max_status_lag_ms";
  static constexpr const char* CONFIG_INTERNAL_DEFAULT_STATUS_TEXT_FIELD =
      "default_status_text";
  static constexpr const char* CONFIG_INTERNAL_NO_FACE_STATUS_TEXT_FIELD =
      "no_face_status_text";
  static constexpr const char* CONFIG_INTERNAL_LOW_SCORE_STATUS_TEXT_FIELD =
      "low_score_status_text";
  static constexpr const char* CONFIG_INTERNAL_MATCHED_PREFIX_FIELD =
      "matched_prefix";

 private:
  enum class TrackStatusType {
    kNoFace,
    kUnknown,
    kLowScore,
    kMatched,
  };

  struct TrackStatusEntry {
    std::string displayText;
    TrackStatusType statusType = TrackStatusType::kNoFace;
    std::string confirmedMatchedLabel;
    std::string pendingMatchedLabel;
    int pendingMatchedCount = 0;
    long long lastMainSeenMs = -1;
    long long latestMainFrameId = -1;
    long long latestMainTimestampMs = -1;
    long long lastStatusTimestampMs = -1;
    std::vector<std::shared_ptr<common::FaceObjectMetadata>> latestFaceObjects;
  };

  using ChannelTrackStatusMap = std::unordered_map<long long, TrackStatusEntry>;

  static long long resolveTimestampMs(
      const std::shared_ptr<common::ObjectMetadata>& objectMetadata);

  void drainStatusPort(int dataPipeId);
  void handleStatusFrame(
      const std::shared_ptr<common::ObjectMetadata>& objectMetadata);
  void attachStatusToMainFrame(
      const std::shared_ptr<common::ObjectMetadata>& objectMetadata);
  void pruneExpiredEntriesLocked(int channelId, long long nowMs);
  std::string buildMatchedText(const std::string& label) const;
  std::string buildStatusText(TrackStatusType statusType) const;
  TrackStatusType resolveStatusType(
      const std::shared_ptr<common::ObjectMetadata>& statusObject,
      std::string& matchedLabel) const;

  int mMainPort = 0;
  int mStatusPort = 1;
  int mTrackTimeoutMs = 3000;
  int mMaxStatusLagMs = 15000;
  std::string mDefaultStatusText = "status: unknown";
  std::string mNoFaceStatusText = "status: no_face";
  std::string mLowScoreStatusText = "status: low_score";
  std::string mMatchedPrefix = "status: ";

  std::mutex mCacheMutex;
  std::unordered_map<int, ChannelTrackStatusMap> mTrackStatusCache;
};

}  // namespace track_status_attach
}  // namespace element
}  // namespace sophon_stream

#endif
