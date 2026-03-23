//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//
#include "faiss.h"

//#if BMCV_VERSION_MAJOR <= 1

#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "common/logger.h"
#include "element_factory.h"

namespace sophon_stream {
namespace element {
namespace faiss {
Faiss::Faiss() {}
Faiss::~Faiss() {
  delete[] db_data;
  delete[] output_dis;
  delete[] output_inx;
  bm_free_device(handle, query_data_dev_mem);
  bm_free_device(handle, db_data_dev_mem);
  bm_free_device(handle, buffer_dev_mem);
  bm_free_device(handle, sorted_similarity_dev_mem);
  bm_free_device(handle, sorted_index_dev_mem);
}

//向量归一化实现
void Faiss::normalizeVector(float* vector, int dims) {
  float norm = 0.0f;
  for (int i = 0; i < dims; ++i) {
    norm += vector[i] * vector[i];
  }
  norm = std::sqrt(norm);
  
  if (norm > 1e-6) { // 避免除以零
    for (int i = 0; i < dims; ++i) {
      vector[i] /= norm;
    }
  }
}

// 批量向量归一化
void Faiss::normalizeVectors(float* vectors, int num, int dims) {
  for (int i = 0; i < num; ++i) {
    normalizeVector(vectors + i * dims, dims);
  }
}

common::ErrorCode Faiss::initInternal(const std::string& json) {
  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;
  STREAM_CHECK(bmcv_faiss_indexflatIP != nullptr, "bmcv_faiss_indexflatIP not support, please update SDK version");
  do {
    auto configure = nlohmann::json::parse(json, nullptr, false);
    if (!configure.is_object()) {
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }

    // 读取相似度阈值配置
    if (configure.find("similarity_threshold") != configure.end()) {
      similarity_threshold = configure["similarity_threshold"].get<float>();
    }

    auto db_data_path =
        configure.find(CONFIG_INTERNAL_DB_DATA_PATH_FILED)->get<std::string>();

    std::vector<float> db_vec(0);
    std::ifstream db_data_file;
    db_data_file.open(db_data_path);
    assert(db_data_file.is_open());
    // 读取文件数据到数组
    if (db_data_file) {
      std::string line;
      int row_count = 0;
      while (std::getline(db_data_file, line)) {
        std::vector<float> row;
        std::stringstream ss(line);
        float val;
        int col_count = 0;
        while (ss >> val) {
          db_vec.push_back(val);
          col_count++;
        }
        if (row_count == 0) {
          vec_dims = col_count;
        }
        row_count++;
      }
      db_vecs_num = row_count;
    }
    db_data_file.close();

    if (db_vecs_num <= 0 || vec_dims <= 0) {
      IVS_ERROR("Faiss database is empty or invalid, db_vecs_num={0:d}, vec_dims={1:d}",
                db_vecs_num, vec_dims);
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }

    if (sort_cnt > db_vecs_num) {
      IVS_WARN("Faiss sort_cnt({0:d}) is larger than db_vecs_num({1:d}), fallback to {1:d}",
               sort_cnt, db_vecs_num);
      sort_cnt = db_vecs_num;
    }
    sort_cnt = std::max(1, sort_cnt);

    auto label_path =
        configure.find(CONFIG_INTERNAL_LABEL_PATH_FILED)->get<std::string>();
    std::ifstream istream;
    istream.open(label_path);
    assert(istream.is_open());
    std::string line;
    while (std::getline(istream, line)) {
      line = line.substr(0, line.length());
      mClassNames.push_back(line);
    }
    istream.close();

    if (mClassNames.empty()) {
      IVS_ERROR("Faiss label file is empty");
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }
    if (static_cast<int>(mClassNames.size()) < db_vecs_num) {
      IVS_WARN("Faiss label count({0:d}) is smaller than db_vecs_num({1:d}), truncate db vectors",
               static_cast<int>(mClassNames.size()), db_vecs_num);
      db_vecs_num = static_cast<int>(mClassNames.size());
    }
    sort_cnt = std::min(sort_cnt, db_vecs_num);

    db_data = new float[db_vecs_num * vec_dims];
    output_dis = new float[query_vecs_num * sort_cnt];
    output_inx = new int[query_vecs_num * sort_cnt];
    std::memcpy(db_data, db_vec.data(), db_vec.size() * sizeof(float));
    
    // 对数据库特征进行归一化
    normalizeVectors(db_data, db_vecs_num, vec_dims);

    bm_dev_request(&handle, 0);
    bm_malloc_device_byte(handle, &buffer_dev_mem,
                          query_vecs_num * db_vecs_num * sizeof(float));
    bm_malloc_device_byte(handle, &sorted_similarity_dev_mem,
                          query_vecs_num * sort_cnt * sizeof(float));
    bm_malloc_device_byte(handle, &sorted_index_dev_mem,
                          query_vecs_num * sort_cnt * sizeof(int));
    bm_malloc_device_byte(handle, &query_data_dev_mem,
                          query_vecs_num * vec_dims * sizeof(float));
    bm_malloc_device_byte(handle, &db_data_dev_mem,
                          db_vecs_num * vec_dims * sizeof(float));
    bm_memcpy_s2d(handle, db_data_dev_mem, db_data);

  } while (false);
  return errorCode;
}

void Faiss::getFaceId(std::shared_ptr<common::RecognizedObjectMetadata> resnetObj) {
  if (resnetObj != nullptr) {
    float* input_data = resnetObj->feature_vector.get();
    
    // 对查询特征进行归一化
    float* normalized_input = new float[vec_dims];
    std::memcpy(normalized_input, input_data, vec_dims * sizeof(float));
    normalizeVector(normalized_input, vec_dims);
    
    {
      std::lock_guard<std::mutex> lock(mutex);
      bm_memcpy_s2d(handle, query_data_dev_mem, normalized_input);
      bmcv_faiss_indexflatIP(handle, query_data_dev_mem, db_data_dev_mem,
                             buffer_dev_mem, sorted_similarity_dev_mem,
                             sorted_index_dev_mem, vec_dims, query_vecs_num,
                             db_vecs_num, sort_cnt, is_transpose, input_dtype,
                             output_dtype);

      bm_memcpy_d2s(handle, output_dis, sorted_similarity_dev_mem);
      bm_memcpy_d2s(handle, output_inx, sorted_index_dev_mem);
    }
    
    delete[] normalized_input;
    
    //相似度阈值判断
    float max_similarity = output_dis[0];
    int label_index = output_inx[0];
    resnetObj->mScores.clear();
    resnetObj->mScores.push_back(max_similarity);
    resnetObj->mTopKLabels.clear();
    
    if (max_similarity >= similarity_threshold &&
        label_index >= 0 &&
        label_index < static_cast<int>(mClassNames.size())) {
      resnetObj->mLabelName = mClassNames[label_index];
      resnetObj->mTopKLabels.push_back(output_inx[0]);
    } else {
      // 未达到阈值的处理
      resnetObj->mLabelName = "unknown";
      resnetObj->mTopKLabels.push_back(-1);
      IVS_DEBUG("Face recognition result below threshold: similarity = {0:.4f}", max_similarity);
    }
  }
}

common::ErrorCode Faiss::doWork(int dataPipeId) {
  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;

  std::vector<int> inputPorts = getInputPorts();
  int inputPort = inputPorts[0];
  int outputPort = 0;
  if (!getSinkElementFlag()) {
    std::vector<int> outputPorts = getOutputPorts();
    int outputPort = outputPorts[0];
  }

  auto data = popInputData(inputPort, dataPipeId);
  while (!data && (getThreadStatus() == ThreadStatus::RUN)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    data = popInputData(inputPort, dataPipeId);
  }
  if (data == nullptr) return common::ErrorCode::SUCCESS;

  auto objectMetadata = std::static_pointer_cast<common::ObjectMetadata>(data);
  int subId = 0;
  // 从resnet取出一个objectMetadata
  // 从data里面取人脸，对一个个人脸做处理，首先需要取出
  for (auto resnetObj : objectMetadata->mRecognizedObjectMetadatas) {
    // mRecognizedObjectMetadatas是一个数组，每个数组包含人脸的框、特征等等,需要做的只是提取特征，填充label
    Faiss::getFaceId(resnetObj);
  }

  int channel_id_internal = objectMetadata->mFrame->mChannelIdInternal;
  int outDataPipeId =
      getSinkElementFlag()
          ? 0
          : (channel_id_internal % getOutputConnectorCapacity(outputPort));
  errorCode = pushOutputData(outputPort, outDataPipeId,
                             std::static_pointer_cast<void>(objectMetadata));
  if (common::ErrorCode::SUCCESS != errorCode) {
    IVS_WARN(
        "Send data fail, element id: {0:d}, output port: {1:d}, data: "
        "{2:p}",
        getId(), outputPort, static_cast<void*>(objectMetadata.get()));
  }
  return errorCode;
}

REGISTER_WORKER("faiss", Faiss)
}  // namespace faiss
}  // namespace element
}  // namespace sophon_stream

//#endif    
