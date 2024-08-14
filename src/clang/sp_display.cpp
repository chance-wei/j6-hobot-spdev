#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "utils_log.h"
#include "vpp_display.h"

#include "sp_display.h"

using namespace std;
using namespace spdev;


void *sp_init_display_module()
{
    return new VPPDisplay();
}

void sp_release_display_module(void *obj)
{
    if (obj != NULL)
    {
        delete static_cast<VPPDisplay *>(obj);
    }
}

int32_t sp_start_display(void *obj, int32_t chn,
        int32_t width, int32_t height)
{
    if (chn != 1) return 0;
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->OpenDisplay(width, height);
    return -1;
}

int32_t sp_stop_display(void *obj)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->Close();
    return -1;
}

int32_t sp_display_set_image(void *obj,
        char *addr, int32_t size, int32_t chn)
{
    ImageFrame frame = {0};

    frame.data[0] = (uint8_t *)addr;
    frame.data_size[0] = size;
    frame.plane_count = 1;

    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->SetImageFrame(&frame);
    return -1;
}

int32_t sp_display_draw_rect(void *obj,
        int32_t x0, int32_t y0, int32_t x1, int32_t y1,
        int32_t chn, int32_t flush, int32_t color, int32_t line_width)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->SetGraphRect(x0, y0, x1, y1, flush, color, line_width);
    return -1;
}

int32_t sp_display_draw_string(void *obj,
        int32_t x, int32_t y, char *str,
        int32_t chn, int32_t flush, int32_t color, int32_t line_width)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->SetGraphWord(x, y, str, flush, (uint32_t)color, line_width);
    return -1;
}

static int32_t exec_cmd_ex(const char *cmd, char* res, int32_t max)
{
    if(cmd == NULL || res == NULL || max <= 0)
            return -1;

    FILE *pp = popen(cmd, "r");
    if(!pp) {
        LOGE_print("[Error] Cannot popen cmd: %s\n", cmd);
        return -1;
    }

    int32_t length;
    char tmp[1024] = {0};

    length = max;
    if(max > 1024) length = 1024;

    while(fgets(tmp, length, pp) != NULL) {
        sscanf(tmp, "%s", res);
    }

    pclose(pp);

    return strlen(res);
}

void sp_get_display_resolution(int32_t *width, int32_t *height)
{
    *width = 1920;
    *height = 1080;
    return;
}
