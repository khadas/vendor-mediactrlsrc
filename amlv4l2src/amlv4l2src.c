/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Amlv4l2src Library
 */

// Copyright (C) 2019 Amlogic, Inc. All rights reserved.
//
// All information contained herein is Amlogic confidential.
//
// This software is provided to you pursuant to Software License
// Agreement (SLA) with Amlogic Inc ("Amlogic"). This software may be
// used only in accordance with the terms of this agreement.
//
// Redistribution and use in source and binary forms, with or without
// modification is strictly prohibited without prior written permission
// from Amlogic.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/videodev2.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <linux/media.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <dirent.h>
#include <sys/sysmacros.h>
#include <sys/socket.h>
#include "amlsrc.h"

#define V4L2_CID_USER_EXT_CAPTURE_BASE (V4L2_CID_USER_BASE + 0x2100)
#define V4L2_CID_EXT_CAPTURE_DIVIDE_FRAMERATE (V4L2_CID_USER_EXT_CAPTURE_BASE + 8)
#define MAX_DEV_LEN 32
#define MEDIA_DEVICE_PATH "/dev"
#define DEVNAME_SIZE 512

typedef void (*Func) (aml_src_t *);

aml_src_t amlsrc;
char g_devname[MAX_DEV_LEN];
bool g_change_framerate = false;

static int change_divide_framerate(int value) {
/* Set 2 as drop 50%; set 0/1 as no dropping */
	int ret;
	struct v4l2_control	control;
  printf("%s, L%d, dev %s, change_frame %d\n", __FUNCTION__, __LINE__, g_devname, g_change_framerate);
  if ((g_devname[0] == '\0')
      || (false == g_change_framerate)
    ) {
    return 0;
  }

  int fd = open(g_devname, O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        printf("open001 \"%s\" error\n", g_devname);
        return -1;
    }
	control.id = V4L2_CID_EXT_CAPTURE_DIVIDE_FRAMERATE;
	control.value = value;

	ret = ioctl(fd, VIDIOC_S_CTRL, &control);
	if (ret) {
		printf("%d VIDIOC_G_EXT_CTRLS fail :%d\n", __LINE__, ret);
    close(fd);
		return -1;
	}
  printf("%s L%d Set framerate div %d success\n", __FUNCTION__, __LINE__, value);
  close(fd);
	return ret;
}


int aml_v4l2src_get_method(aml_src_t *pamlsrc, const char* devtype) {
  if (pamlsrc->initialize &&
      pamlsrc->finalize &&
      pamlsrc->start &&
      pamlsrc->stop) {
    return 0;
  }

  char func_name[32];
  sprintf(func_name, "aml_src_get_%s_method", devtype);
  void *handle;
  handle = dlopen("/usr/lib/libamlsrc.so", RTLD_LAZY);
  if (handle) {
    printf("func_name: %s\n", func_name);
    Func func = (Func)dlsym(handle, func_name);
    if (!func) {
      printf("find function error\n");
      dlclose(handle);
      return -1;
    }
    func(pamlsrc);
    printf("initialize func addr: %p\n", pamlsrc->initialize);
    printf("finalize func addr: %p\n", pamlsrc->finalize);
    printf("start func addr: %p\n", pamlsrc->start);
    printf("stop func addr: %p\n", pamlsrc->stop);
  }

  return 0;
}

// static struct v4l2_capability *cap;
int get_video_type(const char* devname, struct v4l2_capability* cap)
{
//get type by drive name
  int fd = open(devname, O_RDONLY);
    if (fd < 0)
    {
        printf("open001 \"%s\" error\n", devname);
        return -1;
    }
    /* 查询设备属性 */
    // static struct v4l2_capability *cap;
    int ret = ioctl(fd, VIDIOC_QUERYCAP, cap);
    if (ret < 0)

    {
        printf("VIDIOC_QUERYCAP error\n");
        close(fd);
        return -2;
    }

    printf("driver : %s\n", cap->driver);
    printf("device : %s\n", cap->card);
    printf("bus_info : %s\n", cap->bus_info);
    printf("version : %d\n", cap->version);
    close(fd);
    return 0;
}

// Get the default tvin_port for HDMI RX and Screen device
int get_default_tvin_port(const char** devname){
  int ret = 0;
  struct v4l2_capability cap;
  printf("devname : %s\n", *devname);

  ret = get_video_type(*devname, &cap);
  if (0 != ret) {
    printf("Error: get video type failed: %s\n", *devname);
    return -1;
  }

  // HDMI RX
  if (0 == strcmp("vdinvideo",(char*)(cap.driver))) {
    // VPP0(Video)
    return 0;
  }
  // screen
  if (0 == strcmp("amlvideo2",(char*)(cap.driver))) {
    // VPP0(OSD+Video)
    return 0x11000001;
  }

  // VPP0(OSD+Video)
  return 0x11000001;
}

static int get_correspond_media_node(const char *video_filepath, char *media_filepath){
    //get major deice num and minor num
    struct stat device_stat;
    int ret = 0;
    unsigned int video_major_num = 0;
    unsigned int video_minor_num = 0;
    ret = stat(video_filepath, &device_stat);
    if (ret == 0) {
        video_major_num = major(device_stat.st_rdev);
        video_minor_num = minor(device_stat.st_rdev);
    }else{
        printf("%s : Failed to get device %s information\n", __func__, video_filepath);
        return -1;
    }

    //enum mdeia device
    DIR* media_dir = opendir(MEDIA_DEVICE_PATH);
    if (!media_dir) {
        printf("%s : Failed to open device dir\n", __func__ );
        return -1;
    }

    struct dirent* entry;
    while (( entry = readdir(media_dir)) != NULL) {
        if ( entry->d_type == DT_CHR && strncmp(entry->d_name, "media", 5) ==0 ) {
            char media_device[DEVNAME_SIZE] = {0};
            snprintf(media_device, sizeof(media_device), "%s/%s", MEDIA_DEVICE_PATH, entry->d_name);
            printf("%s: get media device %s\n", __func__, media_device );
            struct media_entity_desc info = {0};
            int media_fd = 0;
            int ret = 0;
            media_fd = open(media_device, O_RDONLY);
            if (media_fd < 0) {
                printf("%s: Can't open media device %s\n",
                        __func__, media_device);
                continue;
            }

            ret = ioctl(media_fd, MEDIA_IOC_DEVICE_INFO, &info);
            if (ret < 0) {
                printf("%s: Unable to retrieve media device "
                    "information for device %s \n", __func__,
                    media_device);
                close(media_fd);
                continue;
            }

            for (unsigned int id = 0, ret = 0; ; id = info.id) {
                memset(&info, 0, sizeof(info));
                info.id = id | MEDIA_ENT_ID_FLAG_NEXT;

                ret = ioctl(media_fd, MEDIA_IOC_ENUM_ENTITIES, &info);
                if (ret) {
                    ret = errno != EINVAL ? -errno : 0;
                    printf("%s:error failed to get entity information", __func__ );
                    break;
                }
                else if ( video_major_num == info.v4l.major &&
                          video_minor_num == info.v4l.minor )
                {
                    /* code */
                    printf("%s:info(id %d, type 0x%x, name %s, links %x, v4l-major:%d, v4l2-minor:%d), ret %d\n", __func__, info.id,
                        info.type, info.name, info.links , info.v4l.major, info.v4l.minor,  ret);
                   // media_filepath = media_device;
                    //memcpy(media_filepath, media_device, strlen(media_device)+1);
                    snprintf(media_filepath, sizeof(media_device), "%s", media_device);
                    close(media_fd);
                    closedir(media_dir);
                    return 0;
                }

            }
            close(media_fd);
        }

    }
    closedir(media_dir);
    return -1;
}


int aml_v4l2src_connect(char** devname) {

  int ret = 0;
  struct v4l2_capability cap;
  printf("devname : %s\n", *devname);

  // mipi cam：
  if (0 == strncmp("/dev/media",(char*)(*devname),10)) {
        // aml_v4l2src_get_method(&amlsrc, "cam");
         if (0 == aml_v4l2src_get_method(&amlsrc, "cam")) {
            // *dev_type = 2;//as t7c mipi camera
            char *vidname = amlsrc.initialize(*devname);
            int len = strlen(vidname) + 1;
            free (*devname);
            *devname = malloc(len);
            memcpy(*devname, vidname, len);
            // *devname = g_strdup (amlsrc.initialize(devname));
            // aml_v4l2src_streamon();
            return 2;
          }
  }

  ret = get_video_type(*devname, &cap);
  if (0 != ret) {
    printf("Error: get video type failed: %s\n", *devname);
    return -1;
  }
    printf("strcmp cap.driver: %s\n", cap.driver);

    // usb cam：
  if (0 == strcmp("uvcvideo",(char*)(cap.driver))) {
        // *dev_type = 0;
        return 0;  //as usb
  }
    // mipi cam：
  if (0 == strcmp("ARM-camera-isp",(char*)(cap.driver))) {
        // *dev_type = 1; //as t7 mipi camera
        return 1;
  }

    // hdmi rx：
  if (0 == strcmp("vdinvideo",(char*)(cap.driver)) || 0 == strcmp("vdin_video",(char*)(cap.driver))) {
      if (0 == aml_v4l2src_get_method(&amlsrc, "hdmi")) {
        int dev_len = strlen(*devname) + 1;
        dev_len = (dev_len > MAX_DEV_LEN) ? MAX_DEV_LEN : dev_len;
        memcpy(g_devname, *devname, dev_len);
        g_change_framerate = true;
        change_divide_framerate(2);

        // *dev_type = 3;
        if (amlsrc.initialize)
          amlsrc.initialize(*devname);
        else
          printf("Amlsrc no init interf\n");
        // aml_v4l2src_streamon();
        return 3;
      }
  }
    // hdmi tx：
  if (0 == strcmp("amlvideo2",(char*)(cap.driver))) {
      //  *dev_type = 4;
       return 4;
  }

    if (0 == strcmp("aml-camera",(char*)(cap.driver))) {
        //enum all media node, map to video63
        char media_filepath[DEVNAME_SIZE];
        int ret = 0;
        ret = get_correspond_media_node(*devname, media_filepath);
        printf("%s: media_filepath = %s\n", __func__, media_filepath);
        if (ret < 0) {
            printf("%s: Error get media devnode\n", __func__);
            return -1;
        }
           // *devname = media_filepath;
            if (0 == aml_v4l2src_get_method(&amlsrc, "cam")) {
            char *vidname = amlsrc.initialize(media_filepath);
            printf("%s: finall dev name: %s\n", __func__, *devname);
            return 5;
        }

  }

    printf("unknown case (devname : %s)\n", *devname);

    // *dev_type = -1;
    return -1;

}


void aml_v4l2src_disconnect() {
  if (amlsrc.finalize)
    amlsrc.finalize();
  change_divide_framerate(1);
  g_change_framerate = false;
}


void aml_v4l2src_streamon() {
  if (amlsrc.start)
    amlsrc.start();
}


void aml_v4l2src_streamoff() {
  if (amlsrc.stop)
    amlsrc.stop();
}
