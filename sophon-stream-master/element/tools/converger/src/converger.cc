//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#include "converger.h"

#include <nlohmann/json.hpp>

#include "common/logger.h"
#include "element_factory.h"

namespace sophon_stream {
namespace element {
namespace converger {

Converger::Converger() {}
Converger::~Converger() {}

common::ErrorCode Converger::initInternal(const std::string& json) {
  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;
  do {
    auto configure = nlohmann::json::parse(json, nullptr, false);
    if (!configure.is_object()) {
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }
    int _default_port =
        configure.find(CONFIG_INTERNAL_DEFAULT_PORT_FILED)->get<int>();
    mDefaultPort = _default_port;

    auto maxWaitIt = configure.find(CONFIG_INTERNAL_MAX_WAIT_MS_FIELD);
    if (maxWaitIt != configure.end() && maxWaitIt->is_number_integer()) {
      mMaxWaitMs = maxWaitIt->get<int>();
      if (mMaxWaitMs < 50) mMaxWaitMs = 50;
      if (mMaxWaitMs > 10000) mMaxWaitMs = 10000;
    }
  } while (false);
  return errorCode;
}

common::ErrorCode Converger::doWork(int dataPipeId) {
  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;
  std::vector<int> inputPorts = getInputPorts();
  std::vector<int> outputPorts = getOutputPorts();
  int outputPort = getSinkElementFlag() ? 0 : outputPorts[0];

  // 从所有inputPort中取出数据，并且做判断
  // default_port中取出的数据，放到map里
  auto data = popInputData(mDefaultPort, dataPipeId);
  int retry_times = 0;
  while (!data && (getThreadStatus() == ThreadStatus::RUN) && retry_times < 5) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    data = popInputData(mDefaultPort, dataPipeId);
    ++retry_times;
  }
  if (data != nullptr) {
    auto objectMetadata =
        std::static_pointer_cast<common::ObjectMetadata>(data);
    int channel_id = objectMetadata->mFrame->mChannelIdInternal;
    int frame_id = objectMetadata->mFrame->mSubFrameIdVec.back();
    // lock
    std::unique_lock<std::mutex> lk(mtx);
    mCandidates[channel_id][frame_id] = objectMetadata;
    mCandidateInsertTimes[channel_id][frame_id] = std::chrono::steady_clock::now();
    lk.unlock();
    IVS_DEBUG(
        "data recognized, element_id = {3}, channel_id = {0}, frame_id = {1}, "
        "num_branches = "
        "{2}",
        channel_id, frame_id, objectMetadata->numBranches, getId());
  }

  // 非default_port，取出来之后更新分支数的记录
  for (int inputPort : inputPorts) {
    if (inputPort == mDefaultPort) continue;
    auto subdata = popInputData(inputPort, dataPipeId);
    // 把某个端口给进来的subData都取出来
    while (subdata != nullptr) {
      auto subObj = std::static_pointer_cast<common::ObjectMetadata>(subdata);
      int sub_channel_id = subObj->mFrame->mChannelIdInternal;
      auto sub_frame_id_it = subObj->mFrame->mSubFrameIdVec.end();
      auto sub_frame_id = *(sub_frame_id_it - 2);
      IVS_DEBUG(
          "subData recognized, element_id = {2}, channel_id = {0}, frame_id = "
          "{1}",
          sub_channel_id, sub_frame_id, getId());
      // lock
      std::unique_lock<std::mutex> lk(mtx);
      mBranches[sub_channel_id][sub_frame_id]++;
      lk.unlock();
      IVS_DEBUG(
          "data updated, element_id = {3}, channel_id = {0}, frame_id = {1}, "
          "current "
          "num_branches "
          "= {2}",
          sub_channel_id, sub_frame_id, mBranches[sub_channel_id][sub_frame_id],
          getId());
      subdata = popInputData(inputPort, dataPipeId);
    }
    // if (subdata == nullptr) continue;
  }

  // 遍历map，能弹出去的都弹出去
  for (auto channel_it = mCandidates.begin(); channel_it != mCandidates.end();
       ++channel_it) {
    // 第一层：遍历所有channel
    // 这里需要判断：如果channelId应该和自己这个datapipeId对上，就操作；否则跳过，给其它线程操作
    int channel_id_internal = channel_it->first;
    int dataPipeNums = getThreadNumber();
    if (channel_id_internal % dataPipeNums != dataPipeId) continue;

    for (auto frame_it = mCandidates[channel_id_internal].begin();
         frame_it != mCandidates[channel_id_internal].end();) {
      // 第二层：遍历当前channel下的所有frame，有序
      int frame_id = frame_it->first;
      auto obj = mCandidates[channel_id_internal][frame_id];
      int mergedBranches = mBranches[channel_id_internal][frame_id];
      int expectedBranches = obj->numBranches;
      bool shouldPop = (mergedBranches == expectedBranches);

      if (!shouldPop) {
        auto timeChannelIt = mCandidateInsertTimes.find(channel_id_internal);
        if (timeChannelIt != mCandidateInsertTimes.end()) {
          auto timeIt = timeChannelIt->second.find(frame_id);
          if (timeIt != timeChannelIt->second.end()) {
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() -
                                 timeIt->second)
                                 .count();
            if (elapsedMs > mMaxWaitMs) {
              shouldPop = true;
              IVS_WARN(
                  "Converger wait timeout, force forwarding frame. "
                  "element_id = {0}, channel_id = {1}, frame_id = {2}, "
                  "branches = {3}/{4}, wait_ms = {5}",
                  getId(), channel_id_internal, frame_id, mergedBranches,
                  expectedBranches, static_cast<int>(elapsedMs));
            }
          }
        }
      }

      // 如果可以弹出，则弹出并循环至下一个
      if (shouldPop) {
        IVS_DEBUG(
            "Data converged! Now pop... element_id = {0}, channel_id = {1}, "
            "frame_id = {2}",
            getId(), channel_id_internal, frame_id);
        int outDataPipeId = getSinkElementFlag()
                                ? 0
                                : (channel_id_internal %
                                   getOutputConnectorCapacity(outputPort));
        errorCode = pushOutputData(outputPort, outDataPipeId,
                                   std::static_pointer_cast<void>(obj));
        if (common::ErrorCode::SUCCESS != errorCode) {
          IVS_WARN(
              "Send data fail, element id: {0:d}, output port: {1:d}, data: "
              "{2:p}",
              getId(), outputPort, static_cast<void*>(obj.get()));
        }
        mCandidates[channel_id_internal].erase(frame_it++);
        // delete mBranches[channel_id_internal][frame_id]
        auto branchChannelIt = mBranches.find(channel_id_internal);
        if (branchChannelIt != mBranches.end()) {
          auto channelFrameIt = branchChannelIt->second.find(frame_id);
          if (channelFrameIt != branchChannelIt->second.end()) {
            branchChannelIt->second.erase(channelFrameIt);
          }
        }
        auto timeChannelIt = mCandidateInsertTimes.find(channel_id_internal);
        if (timeChannelIt != mCandidateInsertTimes.end()) {
          auto timeFrameIt = timeChannelIt->second.find(frame_id);
          if (timeFrameIt != timeChannelIt->second.end()) {
            timeChannelIt->second.erase(timeFrameIt);
          }
        }
      } else {
        // 当前帧不可以弹出，为了保证时序性，后续帧也不弹出
        break;
      }
    }
  }
  return errorCode;
}
REGISTER_WORKER("converger", Converger)

}  // namespace converger
}  // namespace element
}  // namespace sophon_stream
