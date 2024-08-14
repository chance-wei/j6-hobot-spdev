/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 16:55:56
 * @LastEditTime: 2023-03-05 16:59:38
 ***************************************************************************/
#include <sys/stat.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "sp_sys.h"
#include "vpp_module.h"

using namespace std;
using namespace spdev;

int sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)
{
    return ((VPPModule *)dst)->BindTo((VPPModule *)src);
}

int sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)
{
    return ((VPPModule *)dst)->UnBind((VPPModule *)src);
}