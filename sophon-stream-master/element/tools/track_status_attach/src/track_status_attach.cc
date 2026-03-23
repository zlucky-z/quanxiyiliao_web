//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#include "track_status_attach.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>

#include "common/logger.h"
#include "element_factory.h"

namespace sophon_stream {
namespace element {
namespace track_status_attach {

namespace {
constexpr long long kMinTimeoutMs = 100;
constexpr long long kMaxTimeoutMs = 30000;
constexpr int kMatchedConfirmFrames = 2;

bool isKnownMatchLabel(const std::string& label) {
  return !label.empty() && label != "unknown" && label != "low_score";
}

std::shared_ptr<common::Frame> cloneFrameMetadata(
    const std::shared_ptr<common::Frame>& frame) {
  if (frame == nullptr) {
    return nullptr;
  }
  return std::make_shared<common::Frame>(*frame);
}

std::shared_ptr<common::PointMetadata> clonePointMetadata(
    const std::shared_ptr<common::PointMetadata>& pointMetadata) {
  if (pointMetadata == nullptr) {
    return nullptr;
  }
  return std::make_shared<common::PointMetadata>(*pointMetadata);
}

std::shared_ptr<common::DetectedObjectMetadata> cloneDetectedObjectMetadata(
    const std::shared_ptr<common::DetectedObjectMetadata>& detectedObject) {
  if (detectedObject == nullptr) {
    return nullptr;
  }

  auto cloned =
      std::make_shared<common::DetectedObjectMetadata>(*detectedObject);
  cloned->mKeyPoints.clear();
  cloned->mKeyPoints.reserve(detectedObject->mKeyPoints.size());
  for (const auto& pointMetadata : detectedObject->mKeyPoints) {
    cloned->mKeyPoints.push_back(clonePointMetadata(pointMetadata));
  }
  return cloned;
}

std::shared_ptr<common::TrackedObjectMetadata> cloneTrackedObjectMetadata(
    const std::shared_ptr<common::TrackedObjectMetadata>& trackedObject) {
  if (trackedObject == nullptr) {
    return nullptr;
  }
  return std::make_shared<common::TrackedObjectMetadata>(*trackedObject);
}

std::shared_ptr<common::FaceObjectMetadata> cloneFaceObjectMetadata(
    const std::shared_ptr<common::FaceObjectMetadata>& faceObject) {
  if (faceObject == nullptr) {
    return nullptr;
  }
  return std::make_shared<common::FaceObjectMetadata>(*faceObject);
}

std::shared_ptr<common::ObjectMetadata> clonePreviewMetadata(
    const std::shared_ptr<common::ObjectMetadata>& objectMetadata) {
  if (objectMetadata == nullptr) {
    return nullptr;
  }

  auto cloned = std::make_shared<common::ObjectMetadata>();
  cloned->mErrorCode = objectMetadata->mErrorCode;
  cloned->mFrame = cloneFrameMetadata(objectMetadata->mFrame);
  cloned->mFilter = objectMetadata->mFilter;
  cloned->mSkipElements = objectMetadata->mSkipElements;
  cloned->tag = objectMetadata->tag;
  cloned->fps = objectMetadata->fps;
  cloned->numBranches = objectMetadata->numBranches;
  cloned->mSubId = objectMetadata->mSubId;
  cloned->mGraphId = objectMetadata->mGraphId;
  cloned->mParentTrackId = objectMetadata->mParentTrackId;
  cloned->is_main = objectMetadata->is_main;
  cloned->resize_vector = objectMetadata->resize_vector;
  cloned->areas = objectMetadata->areas;

  cloned->mDetectedObjectMetadatas.reserve(
      objectMetadata->mDetectedObjectMetadatas.size());
  for (const auto& detObj : objectMetadata->mDetectedObjectMetadatas) {
    cloned->mDetectedObjectMetadatas.push_back(
        cloneDetectedObjectMetadata(detObj));
  }

  cloned->mTrackedObjectMetadatas.reserve(
      objectMetadata->mTrackedObjectMetadatas.size());
  for (const auto& trackObj : objectMetadata->mTrackedObjectMetadatas) {
    cloned->mTrackedObjectMetadatas.push_back(
        cloneTrackedObjectMetadata(trackObj));
  }

  return cloned;
}
}  // namespace

TrackStatusAttach::TrackStatusAttach() {}
TrackStatusAttach::~TrackStatusAttach() {}

common::ErrorCode TrackStatusAttach::initInternal(const std::string& json) {
  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;
  do {
    auto configure = nlohmann::json::parse(json, nullptr, false);
    if (!configure.is_object()) {
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }

    auto mainPortIt = configure.find(CONFIG_INTERNAL_MAIN_PORT_FIELD);
    if (mainPortIt != configure.end() && mainPortIt->is_number_integer()) {
      mMainPort = mainPortIt->get<int>();
    }

    auto statusPortIt = configure.find(CONFIG_INTERNAL_STATUS_PORT_FIELD);
    if (statusPortIt != configure.end() && statusPortIt->is_number_integer()) {
      mStatusPort = statusPortIt->get<int>();
    }

    auto trackTimeoutIt =
        configure.find(CONFIG_INTERNAL_TRACK_TIMEOUT_MS_FIELD);
    if (trackTimeoutIt != configure.end() &&
        trackTimeoutIt->is_number_integer()) {
      mTrackTimeoutMs = std::max(
          static_cast<int>(kMinTimeoutMs),
          std::min(trackTimeoutIt->get<int>(),
                   static_cast<int>(kMaxTimeoutMs)));
    }

    auto maxStatusLagIt =
        configure.find(CONFIG_INTERNAL_MAX_STATUS_LAG_MS_FIELD);
    if (maxStatusLagIt != configure.end() &&
        maxStatusLagIt->is_number_integer()) {
      mMaxStatusLagMs = std::max(
          static_cast<int>(kMinTimeoutMs),
          std::min(maxStatusLagIt->get<int>(),
                   static_cast<int>(kMaxTimeoutMs)));
    }

    auto defaultStatusTextIt =
        configure.find(CONFIG_INTERNAL_DEFAULT_STATUS_TEXT_FIELD);
    if (defaultStatusTextIt != configure.end() &&
        defaultStatusTextIt->is_string() &&
        !defaultStatusTextIt->get<std::string>().empty()) {
      mDefaultStatusText = defaultStatusTextIt->get<std::string>();
    }

    auto noFaceStatusTextIt =
        configure.find(CONFIG_INTERNAL_NO_FACE_STATUS_TEXT_FIELD);
    if (noFaceStatusTextIt != configure.end() &&
        noFaceStatusTextIt->is_string() &&
        !noFaceStatusTextIt->get<std::string>().empty()) {
      mNoFaceStatusText = noFaceStatusTextIt->get<std::string>();
    }

    auto lowScoreStatusTextIt =
        configure.find(CONFIG_INTERNAL_LOW_SCORE_STATUS_TEXT_FIELD);
    if (lowScoreStatusTextIt != configure.end() &&
        lowScoreStatusTextIt->is_string() &&
        !lowScoreStatusTextIt->get<std::string>().empty()) {
      mLowScoreStatusText = lowScoreStatusTextIt->get<std::string>();
    }

    auto matchedPrefixIt =
        configure.find(CONFIG_INTERNAL_MATCHED_PREFIX_FIELD);
    if (matchedPrefixIt != configure.end() && matchedPrefixIt->is_string()) {
      mMatchedPrefix = matchedPrefixIt->get<std::string>();
    }
  } while (false);
  return errorCode;
}

long long TrackStatusAttach::resolveTimestampMs(
    const std::shared_ptr<common::ObjectMetadata>& objectMetadata) {
  const long long fallbackTimestampMs = common::currentTimeMilliseconds();
  if (objectMetadata != nullptr && objectMetadata->mFrame != nullptr) {
    return common::normalizeTimestampToMilliseconds(
        objectMetadata->mFrame->mTimestamp, fallbackTimestampMs);
  }
  return fallbackTimestampMs;
}

std::string TrackStatusAttach::buildMatchedText(const std::string& label) const {
  if (label.empty()) {
    return mDefaultStatusText;
  }
  if (mMatchedPrefix.empty()) {
    return label;
  }
  return mMatchedPrefix + label;
}

std::string TrackStatusAttach::buildStatusText(
    TrackStatusType statusType) const {
  switch (statusType) {
    case TrackStatusType::kNoFace:
      return mNoFaceStatusText;
    case TrackStatusType::kUnknown:
      return mDefaultStatusText;
    case TrackStatusType::kLowScore:
      return mLowScoreStatusText;
    case TrackStatusType::kMatched:
      return mDefaultStatusText;
    default:
      return mNoFaceStatusText;
  }
}

TrackStatusAttach::TrackStatusType TrackStatusAttach::resolveStatusType(
    const std::shared_ptr<common::ObjectMetadata>& statusObject,
    std::string& matchedLabel) const {
  matchedLabel.clear();
  if (statusObject == nullptr) {
    return TrackStatusType::kNoFace;
  }

  bool sawFaceCandidate = !statusObject->mFaceObjectMetadatas.empty();
  bool sawUnknownLabel = false;
  bool sawLowScoreLabel = false;

  auto scanRecognitions =
      [&](const std::vector<std::shared_ptr<common::RecognizedObjectMetadata>>&
              recognizedObjects) {
        for (const auto& recognizedObj : recognizedObjects) {
          if (recognizedObj == nullptr) {
            continue;
          }

          const std::string& label = recognizedObj->mLabelName;
          if (isKnownMatchLabel(label)) {
            matchedLabel = label;
            return true;
          }
          if (label == "low_score") {
            sawLowScoreLabel = true;
          } else if (label == "unknown" || label.empty()) {
            sawUnknownLabel = true;
          }
        }
        return false;
      };

  if (scanRecognitions(statusObject->mRecognizedObjectMetadatas)) {
    return TrackStatusType::kMatched;
  }

  for (const auto& faceSubObject : statusObject->mSubObjectMetadatas) {
    if (faceSubObject == nullptr) {
      continue;
    }
    sawFaceCandidate = true;
    if (scanRecognitions(faceSubObject->mRecognizedObjectMetadatas)) {
      return TrackStatusType::kMatched;
    }
  }

  if (sawLowScoreLabel) {
    return TrackStatusType::kLowScore;
  }
  if (sawUnknownLabel || sawFaceCandidate) {
    return TrackStatusType::kUnknown;
  }
  return TrackStatusType::kNoFace;
}

void TrackStatusAttach::pruneExpiredEntriesLocked(int channelId, long long nowMs) {
  auto channelIt = mTrackStatusCache.find(channelId);
  if (channelIt == mTrackStatusCache.end()) {
    return;
  }

  auto& channelCache = channelIt->second;
  for (auto it = channelCache.begin(); it != channelCache.end();) {
    if (it->second.lastMainSeenMs <= 0 ||
        nowMs - it->second.lastMainSeenMs > mTrackTimeoutMs) {
      it = channelCache.erase(it);
    } else {
      ++it;
    }
  }

  if (channelCache.empty()) {
    mTrackStatusCache.erase(channelIt);
  }
}

void TrackStatusAttach::handleStatusFrame(
    const std::shared_ptr<common::ObjectMetadata>& objectMetadata) {
  if (objectMetadata == nullptr || objectMetadata->mFrame == nullptr) {
    return;
  }

  const int channelId = objectMetadata->mFrame->mChannelIdInternal;
  const long long statusTimestampMs = resolveTimestampMs(objectMetadata);
  const long long trackId = objectMetadata->mParentTrackId;
  if (trackId < 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mCacheMutex);
  pruneExpiredEntriesLocked(channelId, statusTimestampMs);

  auto& entry = mTrackStatusCache[channelId][trackId];
  if (entry.lastStatusTimestampMs > statusTimestampMs) {
    return;
  }

  std::string matchedLabel;
  const TrackStatusType statusType =
      resolveStatusType(objectMetadata, matchedLabel);

  entry.lastStatusTimestampMs = statusTimestampMs;
  entry.latestFaceObjects.clear();
  entry.latestFaceObjects.reserve(objectMetadata->mFaceObjectMetadatas.size());
  for (const auto& faceObject : objectMetadata->mFaceObjectMetadatas) {
    auto clonedFace = cloneFaceObjectMetadata(faceObject);
    if (clonedFace != nullptr) {
      entry.latestFaceObjects.push_back(clonedFace);
    }
  }

  auto resetPendingMatch = [&entry]() {
    entry.pendingMatchedLabel.clear();
    entry.pendingMatchedCount = 0;
  };

  if (statusType == TrackStatusType::kMatched && !matchedLabel.empty()) {
    if (entry.confirmedMatchedLabel == matchedLabel) {
      resetPendingMatch();
      entry.statusType = TrackStatusType::kMatched;
      entry.displayText = buildMatchedText(matchedLabel);
      return;
    }

    if (entry.pendingMatchedLabel == matchedLabel) {
      entry.pendingMatchedCount += 1;
    } else {
      entry.pendingMatchedLabel = matchedLabel;
      entry.pendingMatchedCount = 1;
    }

    if (entry.pendingMatchedCount >= kMatchedConfirmFrames) {
      entry.confirmedMatchedLabel = matchedLabel;
      resetPendingMatch();
      entry.statusType = TrackStatusType::kMatched;
      entry.displayText = buildMatchedText(matchedLabel);
      return;
    }

    if (!entry.confirmedMatchedLabel.empty()) {
      entry.statusType = TrackStatusType::kMatched;
      entry.displayText = buildMatchedText(entry.confirmedMatchedLabel);
    } else {
      entry.statusType = TrackStatusType::kUnknown;
      entry.displayText = buildStatusText(TrackStatusType::kUnknown);
    }
    return;
  }

  resetPendingMatch();
  entry.statusType = statusType;
  entry.displayText = buildStatusText(statusType);
}

void TrackStatusAttach::drainStatusPort(int dataPipeId) {
  auto statusData = popInputData(mStatusPort, dataPipeId);
  while (statusData != nullptr) {
    handleStatusFrame(
        std::static_pointer_cast<common::ObjectMetadata>(statusData));
    statusData = popInputData(mStatusPort, dataPipeId);
  }
}

void TrackStatusAttach::attachStatusToMainFrame(
    const std::shared_ptr<common::ObjectMetadata>& objectMetadata) {
  if (objectMetadata == nullptr || objectMetadata->mFrame == nullptr) {
    return;
  }

  const int channelId = objectMetadata->mFrame->mChannelIdInternal;
  const long long timestampMs = resolveTimestampMs(objectMetadata);
  const long long frameId = objectMetadata->mFrame->mFrameId;

  std::lock_guard<std::mutex> lock(mCacheMutex);
  pruneExpiredEntriesLocked(channelId, timestampMs);

  auto& channelCache = mTrackStatusCache[channelId];
  objectMetadata->mFaceObjectMetadatas.clear();
  objectMetadata->mSubObjectMetadatas.clear();

  for (const auto& trackObject : objectMetadata->mTrackedObjectMetadatas) {
    if (trackObject == nullptr) {
      continue;
    }

    if (trackObject->mTrackId < 0) {
      trackObject->mName.clear();
      trackObject->mRegisteredMatched = false;
      continue;
    }

    auto& entry = channelCache[trackObject->mTrackId];
    entry.lastMainSeenMs = timestampMs;
    entry.latestMainFrameId = frameId;
    entry.latestMainTimestampMs = timestampMs;

    const bool hasFreshStatus =
        entry.lastStatusTimestampMs > 0 &&
        timestampMs - entry.lastStatusTimestampMs <= mMaxStatusLagMs;
    if (!hasFreshStatus) {
      entry.statusType = TrackStatusType::kNoFace;
      entry.displayText = buildStatusText(entry.statusType);
      entry.latestFaceObjects.clear();
    }

    trackObject->mName = entry.displayText.empty() ? mNoFaceStatusText
                                                   : entry.displayText;
    trackObject->mRegisteredMatched =
        entry.statusType == TrackStatusType::kMatched;
    for (const auto& faceObject : entry.latestFaceObjects) {
      auto clonedFace = cloneFaceObjectMetadata(faceObject);
      if (clonedFace != nullptr) {
        objectMetadata->mFaceObjectMetadatas.push_back(clonedFace);
      }
    }
  }

  if (objectMetadata->mFrame->mEndOfStream) {
    mTrackStatusCache.erase(channelId);
  }
}

common::ErrorCode TrackStatusAttach::doWork(int dataPipeId) {
  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;

  drainStatusPort(dataPipeId);

  auto data = popInputData(mMainPort, dataPipeId);
  while (!data && (getThreadStatus() == ThreadStatus::RUN)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    drainStatusPort(dataPipeId);
    data = popInputData(mMainPort, dataPipeId);
  }
  if (data == nullptr) {
    return common::ErrorCode::SUCCESS;
  }

  auto objectMetadata = std::static_pointer_cast<common::ObjectMetadata>(data);
  if (objectMetadata == nullptr || objectMetadata->mFrame == nullptr) {
    return common::ErrorCode::SUCCESS;
  }

  auto outputMetadata = clonePreviewMetadata(objectMetadata);
  if (outputMetadata == nullptr || outputMetadata->mFrame == nullptr) {
    return common::ErrorCode::SUCCESS;
  }

  attachStatusToMainFrame(outputMetadata);

  int outputPort = 0;
  if (!getSinkElementFlag()) {
    std::vector<int> outputPorts = getOutputPorts();
    if (outputPorts.empty()) {
      return common::ErrorCode::SUCCESS;
    }
    outputPort = outputPorts[0];
  }

  const int channelId = outputMetadata->mFrame->mChannelIdInternal;
  const int outDataPipeId =
      getSinkElementFlag() ? 0
                           : (channelId % getOutputConnectorCapacity(outputPort));

  errorCode = pushOutputData(outputPort, outDataPipeId,
                             std::static_pointer_cast<void>(outputMetadata));
  if (common::ErrorCode::SUCCESS != errorCode) {
    IVS_WARN(
        "Send data fail, element id: {0:d}, output port: {1:d}, data: {2:p}",
        getId(), outputPort, static_cast<void*>(outputMetadata.get()));
  }

  return errorCode;
}

REGISTER_WORKER("track_status_attach", TrackStatusAttach)

}  // namespace track_status_attach
}  // namespace element
}  // namespace sophon_stream
