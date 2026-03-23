//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-STREAM is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#include "retinaface_pre_process.h"

#include <iostream>

namespace sophon_stream {
namespace element {
namespace retinaface {

void RetinafacePreProcess::init(std::shared_ptr<RetinafaceContext> context) {}

common::ErrorCode RetinafacePreProcess::preProcess(
    std::shared_ptr<RetinafaceContext> context,
    common::ObjectMetadatas& objectMetadatas) {
  if (objectMetadatas.size() == 0) return common::ErrorCode::SUCCESS;
  initTensors(context, objectMetadatas);
  // write your pre process here
  auto jsonPlanner = context->bgr2rgb ? FORMAT_RGB_PLANAR : FORMAT_BGR_PLANAR;
  int i = 0;
  for (auto& objMetadata : objectMetadatas) {
    if (objMetadata->mFrame->mSpData == nullptr) continue;
    bm_image resized_img;
    bm_image converto_img;
    bm_image image0 = *objMetadata->mFrame->mSpData;
    bm_image image1;
    bm_image image_aligned;
    memset(&resized_img, 0, sizeof(resized_img));
    memset(&converto_img, 0, sizeof(converto_img));
    memset(&image1, 0, sizeof(image1));
    memset(&image_aligned, 0, sizeof(image_aligned));
    bool owns_image1 = false;
    bool owns_image_aligned = false;
    bool owns_resized_img = false;
    bool owns_converto_img = false;
    bool input_mem_attached = false;
    bool input_mem_transferred = false;
    bm_device_mem_t input_mem = bm_mem_null();
    auto cleanup = [&]() {
      if (owns_converto_img) {
        if (input_mem_attached) {
          bm_image_detach(converto_img);
          input_mem_attached = false;
        }
        bm_image_destroy(converto_img);
        owns_converto_img = false;
      }
      if (!input_mem_transferred &&
          input_mem.u.device.device_addr != 0) {
        bm_free_device(context->handle, input_mem);
        input_mem = bm_mem_null();
      }
      if (owns_resized_img) {
        bm_image_destroy(resized_img);
        owns_resized_img = false;
      }
      if (owns_image_aligned) {
        bm_image_destroy(image_aligned);
        owns_image_aligned = false;
      }
      if (owns_image1) {
        bm_image_destroy(image1);
        owns_image1 = false;
      }
    };
    auto fail = [&](const char* stage, bm_status_t ret,
                    common::ErrorCode errorCode) {
      std::cerr << "[RetinafacePreProcess] " << stage
                << " failed, ret=" << ret << std::endl;
      cleanup();
      return errorCode;
    };
    // convert to RGB_PLANAR
    if (image0.image_format != jsonPlanner) {
      bm_image_create(context->handle, image0.height, image0.width, jsonPlanner,
                      image0.data_type, &image1);
      owns_image1 = true;
      auto ret = bm_image_alloc_dev_mem_heap_mask(image1, STREAM_VPU_HEAP_MASK);
      if (ret != BM_SUCCESS) {
        return fail("bm_image_alloc_dev_mem_heap_mask(image1)", ret,
                    common::ErrorCode::ERR_STREAM_MEMORY_ALLOCATION);
      }
      bmcv_image_storage_convert(context->handle, 1, &image0, &image1);
    } else {
      image1 = image0;
    }
    bool need_copy = image1.width & (64 - 1);
    if (need_copy) {
      int stride1[3], stride2[3];
      bm_image_get_stride(image1, stride1);
      stride2[0] = FFALIGN(stride1[0], 64);
      stride2[1] = FFALIGN(stride1[1], 64);
      stride2[2] = FFALIGN(stride1[2], 64);
      bm_image_create(context->bmContext->handle(), image1.height, image1.width,
                      image1.image_format, image1.data_type, &image_aligned,
                      stride2);
      owns_image_aligned = true;

      auto ret = bm_image_alloc_dev_mem_heap_mask(image_aligned, STREAM_VPU_HEAP_MASK);
      if (ret != BM_SUCCESS) {
        return fail("bm_image_alloc_dev_mem_heap_mask(image_aligned)", ret,
                    common::ErrorCode::ERR_STREAM_MEMORY_ALLOCATION);
      }
      bmcv_copy_to_atrr_t copyToAttr;
      memset(&copyToAttr, 0, sizeof(copyToAttr));
      copyToAttr.start_x = 0;
      copyToAttr.start_y = 0;
      copyToAttr.if_padding = 1;
      bmcv_image_copy_to(context->bmContext->handle(), copyToAttr, image1,
                         image_aligned);
    } else {
      image_aligned = image1;
    }
    // #ifdef USE_ASPECT_RATIO
    bool isAlignWidth = false;
    float ratio =
        get_aspect_scaled_ratio(image0.width, image0.height, context->net_w,
                                context->net_h, &isAlignWidth);
    bmcv_padding_atrr_t padding_attr;
    memset(&padding_attr, 0, sizeof(padding_attr));
    padding_attr.dst_crop_sty = 0;
    padding_attr.dst_crop_stx = 0;
    padding_attr.padding_b = 0;
    padding_attr.padding_g = 0;
    padding_attr.padding_r = 0;
    padding_attr.if_memset = 1;
    if (isAlignWidth) {
      padding_attr.dst_crop_h = (image0.height * ratio);
      padding_attr.dst_crop_w = context->net_w;
      padding_attr.dst_crop_sty = 0;
      padding_attr.dst_crop_stx = 0;

    } else {
      padding_attr.dst_crop_h = context->net_h;
      padding_attr.dst_crop_w = (image0.width * ratio);

      int tx1 = (int)((context->net_w - padding_attr.dst_crop_w) / 2);
      padding_attr.dst_crop_sty = 0;
      padding_attr.dst_crop_stx = 0;
    }

    int aligned_net_w = FFALIGN(context->net_w, 64);
    int strides[3] = {aligned_net_w, aligned_net_w, aligned_net_w};
    bm_image_create(context->handle, context->net_h, context->net_w,
                    jsonPlanner, DATA_TYPE_EXT_1N_BYTE, &resized_img, strides);
    owns_resized_img = true;
    auto ret = bm_image_alloc_dev_mem_heap_mask(resized_img, STREAM_VPP_HEAP_MASK);
    if (ret != BM_SUCCESS) {
      return fail("bm_image_alloc_dev_mem_heap_mask(resized_img)", ret,
                  common::ErrorCode::ERR_STREAM_MEMORY_ALLOCATION);
    }
    bmcv_rect_t crop_rect{0, 0, image1.width, image1.height};

    ret = bmcv_image_vpp_convert_padding(context->bmContext->handle(), 1,
                                         image_aligned, &resized_img,
                                         &padding_attr, &crop_rect);

    if (ret != BM_SUCCESS) {
      return fail("bmcv_image_vpp_convert_padding", ret,
                  common::ErrorCode::ERR_IMAGE_DATA);
    }

    if (owns_image1) {
      bm_image_destroy(image1);
      owns_image1 = false;
    }
    if (owns_image_aligned) {
      bm_image_destroy(image_aligned);
      owns_image_aligned = false;
    }

    bm_image_data_format_ext img_dtype = DATA_TYPE_EXT_FLOAT32;
    auto tensor = context->bmNetwork->inputTensor(0);
    if (tensor->get_dtype() == BM_INT8) {
      img_dtype = DATA_TYPE_EXT_1N_BYTE_SIGNED;
    }

    bm_image_create(context->handle, context->net_h, context->net_w,
                    jsonPlanner, img_dtype, &converto_img);
    owns_converto_img = true;

    bm_device_mem_t mem;
    int size_byte = 0;
    bm_image_get_byte_size(converto_img, &size_byte);
    ret = bm_malloc_device_byte_heap(context->handle, &mem, STREAM_NPU_HEAP, size_byte);
    if (ret != BM_SUCCESS) {
      return fail("bm_malloc_device_byte_heap(converto_img)", ret,
                  common::ErrorCode::ERR_STREAM_MEMORY_ALLOCATION);
    }
    input_mem = mem;
    bm_image_attach(converto_img, &mem);
    input_mem_attached = true;

    ret = bmcv_image_convert_to(context->handle, 1, context->converto_attr,
                                &resized_img, &converto_img);
    if (ret != BM_SUCCESS) {
      return fail("bmcv_image_convert_to", ret,
                  common::ErrorCode::ERR_IMAGE_DATA);
    }

    bm_image_destroy(resized_img);
    owns_resized_img = false;
    bm_image_get_device_mem(
        converto_img,
        &objectMetadatas[i]->mInputBMtensors->tensors[0]->device_mem);
    bm_image_detach(converto_img);
    input_mem_attached = false;
    input_mem_transferred = true;
    bm_image_destroy(converto_img);
    owns_converto_img = false;
    i++;
  }
  return common::ErrorCode::SUCCESS;
}

}  // namespace retinaface
}  // namespace element
}  // namespace sophon_stream
