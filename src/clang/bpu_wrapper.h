/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef BPU_WRAPPER_H_
#define BPU_WRAPPER_H_

#include "sp_bpu.h"
#include "dnn/hb_dnn.h"

#ifdef __cplusplus
extern "C"
{
#endif

bpu_module *hb_bpu_predict_init(const char *model_file_name);

int hb_bpu_init_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors);
int hb_bpu_deinit_tensor(hbDNNTensor *tensor, int32_t len);
int hb_bpu_start_predict(bpu_module *bpu_handle, char *frame_buffer);
int hb_bpu_predict_unint(bpu_module *handle);
#ifdef __cplusplus
}
#endif

#endif // BPU_WRAPPER_H_