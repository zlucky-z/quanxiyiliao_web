//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#include "http_push.h"

#include <algorithm>

#include "common/common_defs.h"
#include "common/logger.h"
#include "common/serialize.h"
#include "element_factory.h"

namespace sophon_stream {
namespace element {
namespace http_push {
namespace {

void applyTimeoutMs(httplib::Client& cli, int connection_timeout_ms,
                    int read_timeout_ms, int write_timeout_ms) {
  if (connection_timeout_ms > 0) {
    cli.set_connection_timeout(connection_timeout_ms / 1000,
                               (connection_timeout_ms % 1000) * 1000);
  }
  if (read_timeout_ms > 0) {
    cli.set_read_timeout(read_timeout_ms / 1000,
                         (read_timeout_ms % 1000) * 1000);
  }
  if (write_timeout_ms > 0) {
    cli.set_write_timeout(write_timeout_ms / 1000,
                          (write_timeout_ms % 1000) * 1000);
  }
}

nlohmann::json serializeFrameMetadataOnly(const common::Frame& frame) {
  nlohmann::json j;
  j["mChannelId"] = frame.mChannelId;
  j["mFrameId"] = frame.mFrameId;
  j["mTimestamp"] = frame.mTimestamp;
  j["mWidth"] = frame.mWidth;
  j["mHeight"] = frame.mHeight;
  j["mEndOfStream"] = frame.mEndOfStream;
  return j;
}

nlohmann::json serializeObjectMetadataPayload(
    const std::shared_ptr<common::ObjectMetadata>& obj, bool include_frame_data) {
  nlohmann::json j;
  if (obj == nullptr) {
    return j;
  }

  for (const auto& detObj : obj->mDetectedObjectMetadatas) {
    if (detObj == nullptr) continue;
    j["mDetectedObjectMetadatas"].push_back(*detObj);
  }
  for (const auto& trackObj : obj->mTrackedObjectMetadatas) {
    if (trackObj == nullptr) continue;
    j["mTrackedObjectMetadatas"].push_back(*trackObj);
  }
  for (const auto& poseObj : obj->mPosedObjectMetadatas) {
    if (poseObj == nullptr) continue;
    j["mPosedObjectMetadatas"].push_back(*poseObj);
  }
  for (const auto& recogObj : obj->mRecognizedObjectMetadatas) {
    if (recogObj == nullptr) continue;
    j["mRecognizedObjectMetadatas"].push_back(*recogObj);
  }
  for (const auto& faceObj : obj->mFaceObjectMetadatas) {
    if (faceObj == nullptr) continue;
    j["mFaceObjectMetadata"].push_back(*faceObj);
  }
  j["mFps"] = obj->fps;
  if (obj->mFrame != nullptr) {
    if (include_frame_data) {
      j["mFrame"] = *(obj->mFrame);
    } else {
      j["mFrame"] = serializeFrameMetadataOnly(*(obj->mFrame));
    }
  }
  j["mSubId"] = obj->mSubId;
  j["mGraphId"] = obj->mGraphId;
  j["mParentTrackId"] = obj->mParentTrackId;
  for (const auto& subObj : obj->mSubObjectMetadatas) {
    j["mSubObjectMetadatas"].push_back(
        serializeObjectMetadataPayload(subObj, include_frame_data));
  }
  return j;
}

}  // namespace

HttpPush::HttpPush() {}
HttpPush::~HttpPush() {
  for (auto [k, v] : mapImpl_) {
    v->release();
  }
}

common::ErrorCode HttpPush::initInternal(const std::string& json) {
  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;
  do {
    auto configure = nlohmann::json::parse(json, nullptr, false);
    if (!configure.is_object()) {
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }
    auto ipIt = configure.find(CONFIG_INTERNAL_IP_FILED);
    STREAM_CHECK((ipIt != configure.end() && ipIt->is_string()),
                 "IP must be std::string, please check your http_push element "
                 "configuration file");
    ip_ = ipIt->get<std::string>();
    auto portIt = configure.find(CONFIG_INTERNAL_PORT_FILED);
    STREAM_CHECK((portIt != configure.end() && portIt->is_number_integer()),
                 "Port must be integer, please check your http_push element "
                 "configuration file");
    port_ = portIt->get<int>();

    auto pathIt = configure.find(CONFIG_INTERNAL_PATH_FILED);
    STREAM_CHECK((pathIt != configure.end() && pathIt->is_string()),
                 "Port must be string, please check your http_push element "
                 "configuration file");
    path_ = pathIt->get<std::string>();
    auto includeFrameDataIt =
        configure.find(CONFIG_INTERNAL_INCLUDE_FRAME_DATA_FILED);
    if (includeFrameDataIt == configure.end()) {
      includeFrameData_ = true;
    } else {
      STREAM_CHECK(includeFrameDataIt->is_boolean(),
                   "include_frame_data must be boolean, please check "
                   "your http_push element configuration file");
      includeFrameData_ = includeFrameDataIt->get<bool>();
    }
    auto connectionTimeoutIt =
        configure.find(CONFIG_INTERNAL_CONNECTION_TIMEOUT_MS_FILED);
    if (connectionTimeoutIt != configure.end()) {
      STREAM_CHECK(connectionTimeoutIt->is_number_integer(),
                   "connection_timeout_ms must be integer, please "
                   "check your http_push element configuration file");
      connectionTimeoutMs_ = std::max(0, connectionTimeoutIt->get<int>());
    }
    auto readTimeoutIt =
        configure.find(CONFIG_INTERNAL_READ_TIMEOUT_MS_FILED);
    if (readTimeoutIt != configure.end()) {
      STREAM_CHECK(readTimeoutIt->is_number_integer(),
                   "read_timeout_ms must be integer, please check your "
                   "http_push element configuration file");
      readTimeoutMs_ = std::max(0, readTimeoutIt->get<int>());
    }
	    auto writeTimeoutIt =
	        configure.find(CONFIG_INTERNAL_WRITE_TIMEOUT_MS_FILED);
	    if (writeTimeoutIt != configure.end()) {
      STREAM_CHECK(writeTimeoutIt->is_number_integer(),
                   "write_timeout_ms must be integer, please check "
                   "your http_push element configuration file");
	      writeTimeoutMs_ = std::max(0, writeTimeoutIt->get<int>());
	    }
	    auto latestOnlyIt = configure.find(CONFIG_INTERNAL_LATEST_ONLY_FILED);
	    if (latestOnlyIt != configure.end()) {
	      STREAM_CHECK(latestOnlyIt->is_boolean(),
	                   "latest_only must be boolean, please check your "
	                   "http_push element configuration file");
	      latestOnly_ = latestOnlyIt->get<bool>();
	    }
	    auto minPostIntervalIt =
	        configure.find(CONFIG_INTERNAL_MIN_POST_INTERVAL_MS_FILED);
	    if (minPostIntervalIt != configure.end()) {
	      STREAM_CHECK(minPostIntervalIt->is_number_integer(),
	                   "min_post_interval_ms must be integer, please check "
	                   "your http_push element configuration file");
	      minPostIntervalMs_ = std::max(0, minPostIntervalIt->get<int>());
	    }
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
		    auto schemeIt = configure.find(CONFIG_INTERNAL_SCHEME_FILED);
	    if (schemeIt == configure.end()) {
	        scheme_ = "http";
    } else {
	scheme_ = schemeIt->get<std::string>();  // 获取值
        STREAM_CHECK((scheme_ == "http" || scheme_ == "https"),
                     "Scheme must be http or https, please check your http_push element "
                     "configuration file");
    }

    auto certIt = configure.find(CONFIG_INTERNAL_CERT_FILED);
    if (certIt == configure.end()) {
        cert_ = "";
    } else {
        STREAM_CHECK(certIt->is_string(),
                 "Cert path must be string, please check your http_push element "
                 "configuration file");
        cert_ = certIt->get<std::string>();
    }

    auto keyIt = configure.find(CONFIG_INTERNAL_KEY_FILED);
    if (keyIt == configure.end()) {
        key_ = "";
    } else {
        STREAM_CHECK(keyIt->is_string(),
                 "Key path must be string, please check your http_push element "
                 "configuration file");
        key_ = keyIt->get<std::string>();
    }

    auto cacertIt = configure.find(CONFIG_INTERNAL_CACERT_FILED);
    if (cacertIt == configure.end()) {
        cacert_ = "";
    } else {
        STREAM_CHECK(cacertIt->is_string(),
                 "CACERT path must be string, please check your http_push element "
                 "configuration file");
        cacert_ = cacertIt->get<std::string>();
    }

    auto verifyIt = configure.find(CONFIG_INTERNAL_VERIFY_FILED);
    if (verifyIt == configure.end()) {
        verify_ = false;
    } else {
        verify_ = verifyIt->get<bool>();
    }
#endif

  } while (false);
  return errorCode;
}

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
HttpPushImpl_::HttpPushImpl_(std::string& scheme, std::string& ip, int port, std::string cert, std::string key,
				std::string cacert_, bool verify_, std::string path_, int channel,
	                        int connection_timeout_ms, int read_timeout_ms,
	                        int write_timeout_ms, bool latest_only,
	                        int min_post_interval_ms)
		    : latestOnly(latest_only),
		      minPostIntervalMs(min_post_interval_ms),
		      cli(scheme + "://" + ip + ":" + std::to_string(port), cert, key), cacert(cacert_), verify(verify_), path(path_) {
		  cli.set_ca_cert_path(cacert);
 		  cli.enable_server_certificate_verification(verify);
 		  applyTimeoutMs(cli, connection_timeout_ms, read_timeout_ms,
	                 write_timeout_ms);
	  workThread = std::thread(&HttpPushImpl_::postFunc, this);
	  mFpsProfilerName = "http_push_" + std::to_string(channel) + "_fps";
	  mFpsProfiler.config(mFpsProfilerName, 100);
	}
#else
HttpPushImpl_::HttpPushImpl_(std::string& ip, int port, std::string path_,
                             int channel, int connection_timeout_ms,
                             int read_timeout_ms, int write_timeout_ms,
                             bool latest_only, int min_post_interval_ms)
    : latestOnly(latest_only),
      minPostIntervalMs(min_post_interval_ms),
      cli(ip, port), path(path_) {
  applyTimeoutMs(cli, connection_timeout_ms, read_timeout_ms, write_timeout_ms);
  workThread = std::thread(&HttpPushImpl_::postFunc, this);
  mFpsProfilerName = "http_push_" + std::to_string(channel) + "_fps";
  mFpsProfiler.config(mFpsProfilerName, 100);
}
#endif

void HttpPushImpl_::release() {
  isRunning = false;
  workThread.join();
}

void HttpPushImpl_::postFunc() {
  while (isRunning) {
    auto ptr = popQueue();
    if (ptr == nullptr) {
	      std::this_thread::sleep_for(std::chrono::milliseconds(5));
	      continue;
	    }
	    if (minPostIntervalMs > 0) {
	      auto now = std::chrono::steady_clock::now();
	      if (lastPostTime != std::chrono::steady_clock::time_point::min()) {
	        auto elapsedMs =
	            std::chrono::duration_cast<std::chrono::milliseconds>(
	                now - lastPostTime)
	                .count();
	        if (elapsedMs < minPostIntervalMs) {
	          std::this_thread::sleep_for(
	              std::chrono::milliseconds(minPostIntervalMs - elapsedMs));
	        }
	      }
	    }
	    mFpsProfiler.add(1);
	    auto result = cli.Post(path.c_str(), ptr->dump(), "application/json");
	    lastPostTime = std::chrono::steady_clock::now();
	    if (!result) {
	      IVS_WARN("http_push post failed. path: {0}, error: {1}", path,
               static_cast<int>(result.error()));
    } else if (result->status >= 400) {
      IVS_WARN("http_push post returned status {0}. path: {1}",
               result->status, path);
    }
  }
}

bool HttpPushImpl_::pushQueue(std::shared_ptr<nlohmann::json> j) {
  std::lock_guard<std::mutex> lock(mtx);
  if (latestOnly) {
    while (!objQueue.empty()) {
      objQueue.pop();
    }
    objQueue.push(j);
    return true;
  }
  if (objQueue.size() >= maxQueueLen) return false;
  objQueue.push(j);
  return true;
}

std::shared_ptr<nlohmann::json> HttpPushImpl_::popQueue() {
  std::lock_guard<std::mutex> lock(mtx);
  std::shared_ptr<nlohmann::json> j = nullptr;
  if (objQueue.empty()) return j;
  j = objQueue.front();
  objQueue.pop();
  return j;
}

size_t HttpPushImpl_::getQueueSize() {
  int len = -1;
  {
    std::lock_guard<std::mutex> lock(mtx);
    len = objQueue.size();
  }
  return len;
}

common::ErrorCode HttpPush::doWork(int dataPipeId) {
  std::vector<int> inputPorts = getInputPorts();
  int inputPort = inputPorts[0];
  int outputPort = 0;
  if (!getSinkElementFlag()) {
    std::vector<int> outputPorts = getOutputPorts();
    outputPort = outputPorts[0];
  }

  auto data = popInputData(inputPort, dataPipeId);
  while (!data && (getThreadStatus() == ThreadStatus::RUN)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    data = popInputData(inputPort, dataPipeId);
  }
  if (data == nullptr) return common::ErrorCode::SUCCESS;

  auto objectMetadata = std::static_pointer_cast<common::ObjectMetadata>(data);

  if (!objectMetadata->mFrame->mEndOfStream) {
    nlohmann::json serializedObj =
        serializeObjectMetadataPayload(objectMetadata, includeFrameData_);

    int channel_id = objectMetadata->mFrame->mChannelIdInternal;
    auto implIt = mapImpl_.find(channel_id);
    if (implIt == mapImpl_.end()) {
      std::lock_guard<std::mutex> lock(mapMtx);
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
	      auto httpImpl = std::make_shared<HttpPushImpl_>(scheme_, ip_, port_, cert_, key_,
			      cacert_, verify_, path_, channel_id, connectionTimeoutMs_,
	                      readTimeoutMs_, writeTimeoutMs_, latestOnly_,
	                      minPostIntervalMs_);
#else
	      auto httpImpl =
	          std::make_shared<HttpPushImpl_>(ip_, port_, path_, channel_id,
	                                          connectionTimeoutMs_,
	                                          readTimeoutMs_, writeTimeoutMs_,
	                                          latestOnly_, minPostIntervalMs_);
#endif
      mapImpl_[channel_id] = httpImpl;
      if (!mapImpl_[channel_id]->pushQueue(
              std::make_shared<nlohmann::json>(serializedObj))) {
        IVS_WARN(
            "http_push queue full, drop payload. element id: {0}, channel_id: {1}",
            getId(), channel_id);
      }
    } else {
      if (!implIt->second->pushQueue(
              std::make_shared<nlohmann::json>(serializedObj))) {
        IVS_WARN(
            "http_push queue full, drop payload. element id: {0}, channel_id: {1}",
            getId(), channel_id);
      }
    }
  }

  int channel_id_internal = objectMetadata->mFrame->mChannelIdInternal;
  int outDataPipeId =
      getSinkElementFlag()
          ? 0
          : (channel_id_internal % getOutputConnectorCapacity(outputPort));
  common::ErrorCode errorCode =
      pushOutputData(outputPort, outDataPipeId,
                     std::static_pointer_cast<void>(objectMetadata));
  if (common::ErrorCode::SUCCESS != errorCode) {
    IVS_WARN(
        "Send data fail, element id: {0:d}, output port: {1:d}, data: "
        "{2:p}",
        getId(), outputPort, static_cast<void*>(objectMetadata.get()));
  }
  return common::ErrorCode::SUCCESS;
}

REGISTER_WORKER("http_push", HttpPush)
}  // namespace http_push
}  // namespace element
}  // namespace sophon_stream
