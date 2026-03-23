//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#include "face_track_matcher.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <thread>

#include <nlohmann/json.hpp>

#include "common/logger.h"
#include "element_factory.h"

namespace sophon_stream {
namespace element {
namespace face_track_matcher {

namespace {
constexpr long long kMinTimeoutMs = 100;
constexpr long long kMaxTimeoutMs = 30000;
constexpr long long kMaxFrameGap = 3;

int intersectionArea(const common::Rectangle<int>& a,
                     const common::Rectangle<int>& b) {
  const int left = std::max(a.mX, b.mX);
  const int top = std::max(a.mY, b.mY);
  const int right = std::min(a.mX + a.mWidth, b.mX + b.mWidth);
  const int bottom = std::min(a.mY + a.mHeight, b.mY + b.mHeight);
  if (right <= left || bottom <= top) {
    return 0;
  }
  return (right - left) * (bottom - top);
}

bool containsPoint(const common::Rectangle<int>& box, int x, int y) {
  return x >= box.mX && y >= box.mY && x < box.mX + box.mWidth &&
         y < box.mY + box.mHeight;
}

std::shared_ptr<common::Frame> cloneFrameMetadata(
    const std::shared_ptr<common::Frame>& frame) {
  if (frame == nullptr) {
    return nullptr;
  }
  return std::make_shared<common::Frame>(*frame);
}

std::shared_ptr<common::FaceObjectMetadata> cloneFaceObjectMetadata(
    const std::shared_ptr<common::FaceObjectMetadata>& faceObject) {
  if (faceObject == nullptr) {
    return nullptr;
  }
  return std::make_shared<common::FaceObjectMetadata>(*faceObject);
}
}  // namespace

FaceTrackMatcher::FaceTrackMatcher() {}
FaceTrackMatcher::~FaceTrackMatcher() {}

common::ErrorCode FaceTrackMatcher::initInternal(const std::string& json) {
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

    auto facePortIt = configure.find(CONFIG_INTERNAL_FACE_PORT_FIELD);
    if (facePortIt != configure.end() && facePortIt->is_number_integer()) {
      mFacePort = facePortIt->get<int>();
    }

    auto faceCacheMaxAgeIt =
        configure.find(CONFIG_INTERNAL_FACE_CACHE_MAX_AGE_MS_FIELD);
    if (faceCacheMaxAgeIt != configure.end() &&
        faceCacheMaxAgeIt->is_number_integer()) {
      mFaceCacheMaxAgeMs = std::max(
          static_cast<int>(kMinTimeoutMs),
          std::min(faceCacheMaxAgeIt->get<int>(),
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

    auto minOverlapRatioIt =
        configure.find(CONFIG_INTERNAL_MIN_OVERLAP_RATIO_FIELD);
    if (minOverlapRatioIt != configure.end() &&
        minOverlapRatioIt->is_number()) {
      mMinFacePersonOverlapRatio = std::max(
          0.0f, std::min(minOverlapRatioIt->get<float>(), 1.0f));
    }
  } while (false);
  return errorCode;
}

long long FaceTrackMatcher::resolveTimestampMs(
    const std::shared_ptr<common::ObjectMetadata>& objectMetadata) {
  const long long fallbackTimestampMs = common::currentTimeMilliseconds();
  if (objectMetadata != nullptr && objectMetadata->mFrame != nullptr) {
    return common::normalizeTimestampToMilliseconds(
        objectMetadata->mFrame->mTimestamp, fallbackTimestampMs);
  }
  return fallbackTimestampMs;
}

void FaceTrackMatcher::pruneExpiredEntriesLocked(int channelId,
                                                 long long nowMs) {
  auto channelIt = mMainFrameCache.find(channelId);
  if (channelIt != mMainFrameCache.end()) {
    auto& channelCache = channelIt->second;
    for (auto it = channelCache.begin(); it != channelCache.end();) {
      if (it->second.timestampMs < 0 ||
          nowMs - it->second.timestampMs > mFaceCacheMaxAgeMs) {
        it = channelCache.erase(it);
      } else {
        ++it;
      }
    }

    if (channelCache.empty()) {
      mMainFrameCache.erase(channelIt);
    }
  }

  auto pendingFaceIt = mPendingFaceFrameCache.find(channelId);
  if (pendingFaceIt == mPendingFaceFrameCache.end()) {
    return;
  }

  auto& pendingFaceCache = pendingFaceIt->second;
  for (auto it = pendingFaceCache.begin(); it != pendingFaceCache.end();) {
    if (it->second.timestampMs < 0 ||
        nowMs - it->second.timestampMs > mFaceCacheMaxAgeMs) {
      it = pendingFaceCache.erase(it);
    } else {
      ++it;
    }
  }

  if (pendingFaceCache.empty()) {
    mPendingFaceFrameCache.erase(pendingFaceIt);
  }
}

void FaceTrackMatcher::cacheMainFrame(
    const std::shared_ptr<common::ObjectMetadata>& objectMetadata) {
  if (objectMetadata == nullptr || objectMetadata->mFrame == nullptr) {
    return;
  }

  const int channelId = objectMetadata->mFrame->mChannelIdInternal;
  const long long timestampMs = resolveTimestampMs(objectMetadata);

  std::lock_guard<std::mutex> lock(mCacheMutex);
  auto& channelCache = mMainFrameCache[channelId];
  channelCache[objectMetadata->mFrame->mFrameId] =
      MainFrameCacheEntry{timestampMs, objectMetadata->mFrame->mFrameId,
                          objectMetadata};
  pruneExpiredEntriesLocked(channelId, timestampMs);
}

void FaceTrackMatcher::cachePendingFaceFrameLocked(
    const std::shared_ptr<common::ObjectMetadata>& objectMetadata,
    long long timestampMs) {
  if (objectMetadata == nullptr || objectMetadata->mFrame == nullptr ||
      objectMetadata->mFaceObjectMetadatas.empty()) {
    return;
  }

  const int channelId = objectMetadata->mFrame->mChannelIdInternal;
  auto& channelCache = mPendingFaceFrameCache[channelId];
  channelCache[objectMetadata->mFrame->mFrameId] =
      PendingFaceFrameEntry{timestampMs, objectMetadata->mFrame->mFrameId,
                            objectMetadata};
  pruneExpiredEntriesLocked(channelId, timestampMs);
}

const FaceTrackMatcher::MainFrameCacheEntry*
FaceTrackMatcher::findBestMainFrameLocked(int channelId,
                                          long long frameId,
                                          long long timestampMs) {
  pruneExpiredEntriesLocked(channelId, timestampMs);

  auto channelIt = mMainFrameCache.find(channelId);
  if (channelIt == mMainFrameCache.end() || channelIt->second.empty()) {
    return nullptr;
  }

  auto& channelCache = channelIt->second;
  const MainFrameCacheEntry* bestEntry = nullptr;
  long long bestFrameGap = std::numeric_limits<long long>::max();
  long long bestTimeGap = std::numeric_limits<long long>::max();

  auto evaluateEntry = [&](const MainFrameCacheEntry& entry) {
    if (entry.objectMetadata == nullptr || entry.objectMetadata->mFrame == nullptr ||
        entry.objectMetadata->mTrackedObjectMetadatas.empty() ||
        entry.objectMetadata->mDetectedObjectMetadatas.empty()) {
      return;
    }

    const long long frameGap = std::llabs(entry.frameId - frameId);
    if (frameGap > kMaxFrameGap) {
      return;
    }

    const long long timeGap = std::llabs(entry.timestampMs - timestampMs);
    if (timeGap > mMaxStatusLagMs) {
      return;
    }

    if (bestEntry == nullptr || frameGap < bestFrameGap ||
        (frameGap == bestFrameGap && timeGap < bestTimeGap) ||
        (frameGap == bestFrameGap && timeGap == bestTimeGap &&
         entry.frameId > bestEntry->frameId)) {
      bestEntry = &entry;
      bestFrameGap = frameGap;
      bestTimeGap = timeGap;
    }
  };

  auto exactIt = channelCache.find(frameId);
  if (exactIt != channelCache.end()) {
    evaluateEntry(exactIt->second);
  }

  auto upperIt = channelCache.lower_bound(frameId);
  if (upperIt != channelCache.end()) {
    evaluateEntry(upperIt->second);
  }
  if (upperIt != channelCache.begin()) {
    auto prevIt = upperIt;
    --prevIt;
    evaluateEntry(prevIt->second);
  }

  return bestEntry;
}

const FaceTrackMatcher::PendingFaceFrameEntry*
FaceTrackMatcher::findBestPendingFaceFrameLocked(int channelId,
                                                 long long frameId,
                                                 long long timestampMs) {
  pruneExpiredEntriesLocked(channelId, timestampMs);

  auto channelIt = mPendingFaceFrameCache.find(channelId);
  if (channelIt == mPendingFaceFrameCache.end() ||
      channelIt->second.empty()) {
    return nullptr;
  }

  auto& channelCache = channelIt->second;
  const PendingFaceFrameEntry* bestEntry = nullptr;
  long long bestFrameGap = std::numeric_limits<long long>::max();
  long long bestTimeGap = std::numeric_limits<long long>::max();

  auto evaluateEntry = [&](const PendingFaceFrameEntry& entry) {
    if (entry.objectMetadata == nullptr || entry.objectMetadata->mFrame == nullptr ||
        entry.objectMetadata->mFaceObjectMetadatas.empty()) {
      return;
    }

    const long long frameGap = std::llabs(entry.frameId - frameId);
    if (frameGap > kMaxFrameGap) {
      return;
    }

    const long long timeGap = std::llabs(entry.timestampMs - timestampMs);
    if (timeGap > mMaxStatusLagMs) {
      return;
    }

    if (bestEntry == nullptr || frameGap < bestFrameGap ||
        (frameGap == bestFrameGap && timeGap < bestTimeGap) ||
        (frameGap == bestFrameGap && timeGap == bestTimeGap &&
         entry.frameId > bestEntry->frameId)) {
      bestEntry = &entry;
      bestFrameGap = frameGap;
      bestTimeGap = timeGap;
    }
  };

  auto exactIt = channelCache.find(frameId);
  if (exactIt != channelCache.end()) {
    evaluateEntry(exactIt->second);
  }

  auto upperIt = channelCache.lower_bound(frameId);
  if (upperIt != channelCache.end()) {
    evaluateEntry(upperIt->second);
  }
  if (upperIt != channelCache.begin()) {
    auto prevIt = upperIt;
    --prevIt;
    evaluateEntry(prevIt->second);
  }

  return bestEntry;
}

common::ErrorCode FaceTrackMatcher::emitMatchedFaces(
    const std::shared_ptr<common::ObjectMetadata>& faceObjectMetadata,
    const MainFrameCacheEntry& mainFrameEntry) {
  if (faceObjectMetadata == nullptr || faceObjectMetadata->mFrame == nullptr ||
      mainFrameEntry.objectMetadata == nullptr ||
      mainFrameEntry.objectMetadata->mFrame == nullptr) {
    return common::ErrorCode::SUCCESS;
  }

  int outputPort = 0;
  if (!getSinkElementFlag()) {
    std::vector<int> outputPorts = getOutputPorts();
    if (outputPorts.empty()) {
      return common::ErrorCode::SUCCESS;
    }
    outputPort = outputPorts[0];
  }

  struct MatchCandidate {
    bool matchedByContainment = false;
    int containArea = std::numeric_limits<int>::max();
    float overlapRatio = 0.0f;
    float faceScore = 0.0f;
    long long trackId = -1;
    std::shared_ptr<common::FaceObjectMetadata> faceObject;
  };

  std::unordered_map<long long, MatchCandidate> bestMatches;
  const size_t maxTrackCount =
      std::min(mainFrameEntry.objectMetadata->mTrackedObjectMetadatas.size(),
               mainFrameEntry.objectMetadata->mDetectedObjectMetadatas.size());

  for (const auto& faceObject :
       faceObjectMetadata->mFaceObjectMetadatas) {
    if (faceObject == nullptr) {
      continue;
    }

    common::Rectangle<int> faceBox{
        faceObject->left,
        faceObject->top,
        std::max(faceObject->right - faceObject->left + 1, 0),
        std::max(faceObject->bottom - faceObject->top + 1, 0)};
    if (faceBox.mWidth <= 0 || faceBox.mHeight <= 0) {
      continue;
    }

    const int faceCenterX = faceObject->left + faceBox.mWidth / 2;
    const int faceCenterY = faceObject->top + faceBox.mHeight / 2;
    MatchCandidate candidate;
    candidate.faceScore = faceObject->score;
    candidate.faceObject = cloneFaceObjectMetadata(faceObject);

    for (size_t trackIndex = 0; trackIndex < maxTrackCount; ++trackIndex) {
      const auto& trackObject =
          mainFrameEntry.objectMetadata->mTrackedObjectMetadatas[trackIndex];
      const auto& detectedObject =
          mainFrameEntry.objectMetadata->mDetectedObjectMetadatas[trackIndex];
      if (trackObject == nullptr || detectedObject == nullptr ||
          trackObject->mTrackId < 0) {
        continue;
      }

      const common::Rectangle<int>& personBox = detectedObject->mBox;
      const int personArea = personBox.mWidth * personBox.mHeight;
      if (personArea <= 0) {
        continue;
      }

      if (containsPoint(personBox, faceCenterX, faceCenterY)) {
        if (!candidate.matchedByContainment || personArea < candidate.containArea) {
          candidate.matchedByContainment = true;
          candidate.containArea = personArea;
          candidate.trackId = trackObject->mTrackId;
        }
        continue;
      }

      if (candidate.matchedByContainment) {
        continue;
      }

      const int overlapArea = intersectionArea(personBox, faceBox);
      const float overlapRatio =
          static_cast<float>(overlapArea) /
          static_cast<float>(faceBox.mWidth * faceBox.mHeight);
      if (overlapRatio >= mMinFacePersonOverlapRatio &&
          overlapRatio > candidate.overlapRatio) {
        candidate.overlapRatio = overlapRatio;
        candidate.trackId = trackObject->mTrackId;
      }
    }

    if (candidate.trackId < 0 || candidate.faceObject == nullptr) {
      continue;
    }

    auto existingIt = bestMatches.find(candidate.trackId);
    auto shouldReplace = [&](const MatchCandidate& current,
                             const MatchCandidate& existing) {
      if (current.matchedByContainment != existing.matchedByContainment) {
        return current.matchedByContainment;
      }
      if (current.matchedByContainment) {
        if (current.containArea != existing.containArea) {
          return current.containArea < existing.containArea;
        }
      } else if (std::fabs(current.overlapRatio - existing.overlapRatio) >
                 1e-6f) {
        return current.overlapRatio > existing.overlapRatio;
      }
      return current.faceScore > existing.faceScore;
    };

    if (existingIt == bestMatches.end() ||
        shouldReplace(candidate, existingIt->second)) {
      bestMatches[candidate.trackId] = candidate;
    }
  }

  common::ErrorCode lastError = common::ErrorCode::SUCCESS;
  const int channelId = faceObjectMetadata->mFrame->mChannelIdInternal;
  const int outDataPipeId =
      getSinkElementFlag() ? 0
                           : (channelId %
                              getOutputConnectorCapacity(outputPort));

  for (const auto& matchItem : bestMatches) {
    const long long trackId = matchItem.first;
    const MatchCandidate& matchCandidate = matchItem.second;
    if (trackId < 0 || matchCandidate.faceObject == nullptr) {
      continue;
    }

    auto outputMetadata = std::make_shared<common::ObjectMetadata>();
    outputMetadata->mErrorCode = faceObjectMetadata->mErrorCode;
    outputMetadata->mFrame = cloneFrameMetadata(faceObjectMetadata->mFrame);
    outputMetadata->mFilter = faceObjectMetadata->mFilter;
    outputMetadata->mSkipElements = faceObjectMetadata->mSkipElements;
    outputMetadata->tag = faceObjectMetadata->tag;
    outputMetadata->fps = faceObjectMetadata->fps;
    outputMetadata->mGraphId = faceObjectMetadata->mGraphId;
    outputMetadata->mParentTrackId = trackId;
    outputMetadata->mFaceObjectMetadatas.push_back(matchCandidate.faceObject);

    auto& subFrameId = mSubFrameIdMap[channelId];
    outputMetadata->mFrame->mSubFrameIdVec.push_back(subFrameId++);

    lastError = pushOutputData(outputPort, outDataPipeId,
                               std::static_pointer_cast<void>(outputMetadata));
    if (lastError != common::ErrorCode::SUCCESS) {
      IVS_WARN(
          "Drop matched face to keep pipeline moving, element id: {0:d}, "
          "channel_id: {1:d}, frame_id: {2:d}, track_id: {3:d}",
          getId(), channelId,
          static_cast<int>(outputMetadata->mFrame->mFrameId),
          static_cast<int>(trackId));
    }
  }

  return lastError;
}

common::ErrorCode FaceTrackMatcher::emitEndOfStream(
    const std::shared_ptr<common::ObjectMetadata>& mainObjectMetadata) {
  if (mainObjectMetadata == nullptr || mainObjectMetadata->mFrame == nullptr) {
    return common::ErrorCode::SUCCESS;
  }

  int outputPort = 0;
  if (!getSinkElementFlag()) {
    std::vector<int> outputPorts = getOutputPorts();
    if (outputPorts.empty()) {
      return common::ErrorCode::SUCCESS;
    }
    outputPort = outputPorts[0];
  }

  auto outputMetadata = std::make_shared<common::ObjectMetadata>();
  outputMetadata->mErrorCode = mainObjectMetadata->mErrorCode;
  outputMetadata->mFrame = cloneFrameMetadata(mainObjectMetadata->mFrame);
  outputMetadata->mFilter = mainObjectMetadata->mFilter;
  outputMetadata->mSkipElements = mainObjectMetadata->mSkipElements;
  outputMetadata->tag = mainObjectMetadata->tag;
  outputMetadata->fps = mainObjectMetadata->fps;
  outputMetadata->mGraphId = mainObjectMetadata->mGraphId;

  const int channelId = mainObjectMetadata->mFrame->mChannelIdInternal;
  auto& subFrameId = mSubFrameIdMap[channelId];
  outputMetadata->mFrame->mSubFrameIdVec.push_back(subFrameId++);

  const int outDataPipeId =
      getSinkElementFlag() ? 0
                           : (channelId %
                              getOutputConnectorCapacity(outputPort));
  return pushOutputData(outputPort, outDataPipeId,
                        std::static_pointer_cast<void>(outputMetadata));
}

common::ErrorCode FaceTrackMatcher::doWork(int dataPipeId) {
  while (getThreadStatus() == ThreadStatus::RUN) {
    auto faceData = popInputData(mFacePort, dataPipeId);
    if (faceData != nullptr) {
      auto faceObjectMetadata =
          std::static_pointer_cast<common::ObjectMetadata>(faceData);
      if (faceObjectMetadata == nullptr || faceObjectMetadata->mFrame == nullptr ||
          faceObjectMetadata->mFrame->mEndOfStream) {
        return common::ErrorCode::SUCCESS;
      }

      const long long frameId = faceObjectMetadata->mFrame->mFrameId;
      const long long timestampMs = resolveTimestampMs(faceObjectMetadata);
      const int channelId = faceObjectMetadata->mFrame->mChannelIdInternal;

      std::lock_guard<std::mutex> lock(mCacheMutex);
      const MainFrameCacheEntry* mainFrameEntry =
          findBestMainFrameLocked(channelId, frameId, timestampMs);
      if (mainFrameEntry == nullptr) {
        cachePendingFaceFrameLocked(faceObjectMetadata, timestampMs);
        return common::ErrorCode::SUCCESS;
      }
      return emitMatchedFaces(faceObjectMetadata, *mainFrameEntry);
    }

    auto mainData = popInputData(mMainPort, dataPipeId);
    if (mainData != nullptr) {
      auto mainObjectMetadata =
          std::static_pointer_cast<common::ObjectMetadata>(mainData);
      if (mainObjectMetadata == nullptr || mainObjectMetadata->mFrame == nullptr) {
        return common::ErrorCode::SUCCESS;
      }

      if (mainObjectMetadata->mFrame->mEndOfStream) {
        {
          std::lock_guard<std::mutex> lock(mCacheMutex);
          mMainFrameCache.erase(mainObjectMetadata->mFrame->mChannelIdInternal);
          mPendingFaceFrameCache.erase(
              mainObjectMetadata->mFrame->mChannelIdInternal);
        }
        return emitEndOfStream(mainObjectMetadata);
      }

      cacheMainFrame(mainObjectMetadata);
      const long long frameId = mainObjectMetadata->mFrame->mFrameId;
      const long long timestampMs = resolveTimestampMs(mainObjectMetadata);
      const int channelId = mainObjectMetadata->mFrame->mChannelIdInternal;

      std::lock_guard<std::mutex> lock(mCacheMutex);
      const MainFrameCacheEntry* mainFrameEntry =
          findBestMainFrameLocked(channelId, frameId, timestampMs);
      const PendingFaceFrameEntry* pendingFaceEntry =
          findBestPendingFaceFrameLocked(channelId, frameId, timestampMs);
      if (mainFrameEntry != nullptr && pendingFaceEntry != nullptr) {
        const long long pendingFrameId = pendingFaceEntry->frameId;
        common::ErrorCode errorCode =
            emitMatchedFaces(pendingFaceEntry->objectMetadata, *mainFrameEntry);
        auto pendingChannelIt = mPendingFaceFrameCache.find(channelId);
        if (pendingChannelIt != mPendingFaceFrameCache.end()) {
          pendingChannelIt->second.erase(pendingFrameId);
          if (pendingChannelIt->second.empty()) {
            mPendingFaceFrameCache.erase(pendingChannelIt);
          }
        }
        return errorCode;
      }
      return common::ErrorCode::SUCCESS;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  return common::ErrorCode::SUCCESS;
}

REGISTER_WORKER("face_track_matcher", FaceTrackMatcher)

}  // namespace face_track_matcher
}  // namespace element
}  // namespace sophon_stream
