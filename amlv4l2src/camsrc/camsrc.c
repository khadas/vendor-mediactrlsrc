/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Camsrc: Amlsrc Implementation
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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#ifdef HAVE_LIBUDEV
#include <libudev.h>
#endif

#include "mediactrl_common.h"
#include "mediactrl_log.h"

static char *server_socket = DEFAULT_SERVER_SOCKET0;
static int client_sockfd = -1;
static char vdevname_buffer[32] = {0};

int cam_src_select_socket(const char *filepath) {
  if (strstr(filepath, "media")) {
    if (access(DEFAULT_SERVER_SOCKET0, F_OK) == 0) {
        log_debug("Socket0 working");
        if (access(DEFAULT_SERVER_SOCKET1, F_OK) == 0) {
            log_debug("Socket1 working");
            log_debug("FATAL: bug in camsrc\n \
            Maybe socket remain after camsrc end, or out of max support\n \
            current max support dual mipi camera, please debug and ");
            return -2;
        }else {
            log_debug("select socket1");
            server_socket = DEFAULT_SERVER_SOCKET1;
            return 0;
        }
    } else {
        log_debug("select socket0");
        server_socket = DEFAULT_SERVER_SOCKET0;
        return 0;
    }
  } else {
    log_debug("Not media src: %s", filepath);
    return -1;
  }

  return -3;
}

static void
cam_src_obtain_devname(const char *filepath) {
  if (0 != cam_src_select_socket(filepath)) {
    log_debug("not supported media device name ...");
    return;
  }

  pid_t pid;
  pid = fork();
  if (pid < 0) {
    log_debug("fork error");
    exit(1);
  }
  if (pid == 0) {
    prctl(PR_SET_PDEATHSIG, SIGTERM);
    /* call execl to startup camctrl */
    execl("/usr/bin/camctrl", "camctrl", "-m", filepath, "-c", "2", "-s", server_socket, NULL);
  }

  while (true) {
    if (!access(server_socket, F_OK))
      break;
    else
      continue;
  }

  /* temporarily */
  usleep(200000);

  client_sockfd = udp_sock_create(server_socket);
  udp_sock_recv(
    client_sockfd,
    vdevname_buffer,
    sizeof(vdevname_buffer)
  );
}
#ifdef HAVE_LIBUDEV
static int cam_get_devname_udev(struct udev *udev,
        struct media_entity_desc *info)
{
    struct udev_device *device;
    dev_t devnum;
    const char *p;
    int ret = -ENODEV;

    if (udev == NULL)
        return -EINVAL;

    devnum = makedev(info->v4l.major, info->v4l.minor);
    log_debug("Looking up v4l[%u:%u] device: %u:%u\n",
          info->v4l.major, info->v4l.minor,
          major(devnum), minor(devnum));
    device = udev_device_new_from_devnum(udev, 'c', devnum);
    if (device) {
        p = udev_device_get_devnode(device);
        if (p) {
            sprintf(vdevname_buffer, "%s", p);
            ret = 0;
        }
    }

    udev_device_unref(device);

    return ret;
}
#endif

static int cam_get_devname_sysfs(struct media_entity_desc *info)
{
    struct stat devstat;
    char devname[32];
    char sysname[32];
    char target[1024];
    char *p;
    int ret;

    sprintf(sysname, "/sys/dev/char/%u:%u", info->v4l.major,
        info->v4l.minor);
    ret = readlink(sysname, target, sizeof(target) - 1);
    if (ret < 0)
        return -errno;

    target[ret] = '\0';
    p = strrchr(target, '/');
    if (p == NULL)
        return -EINVAL;

    sprintf(devname, "/dev/%s", p + 1);
    ret = stat(devname, &devstat);
    if (ret < 0)
        return -errno;

    /* Sanity check: udev might have reordered the device nodes.
     * Make sure the major/minor match. We should really use
     * libudev.
     */
    if (major(devstat.st_rdev) == info->v4l.major &&
        minor(devstat.st_rdev) == info->v4l.minor)
        sprintf(vdevname_buffer, "%s", devname);
    log_debug("%s: get devname %s, via: %s\n",
          __func__, devname, sysname);
    return 0;
}

static bool carm_src_is_usb(const char* filepath) {
    int fd = 0;
    int ret = 0;
    struct media_entity_desc info = {0};

    fd = open(filepath, O_RDONLY);
    if (fd < 0) {
      log_error("%s: Can't open media device %s, error: %s\n",
            __func__, filepath, strerror(errno));
      return false;
    }

    ret = ioctl(fd, MEDIA_IOC_DEVICE_INFO, &info);
    if (ret < 0) {
        log_error("%s: Unable to retrieve media device "
              "information for device %s (%s)\n", __func__,
              filepath, strerror(errno));
        close(fd);
        return false;
    }

#ifdef HAVE_LIBUDEV
    struct udev *udev = NULL;
    udev = udev_new();
    if (udev == NULL) {
        log_error("%s:new udev failed\n", __func__);
        close(fd);
        return false;
    }
#endif

for (unsigned int id = 0, ret = 0; ; id = info.id) {
    memset(&info, 0, sizeof(info));
    info.id = id | MEDIA_ENT_ID_FLAG_NEXT;

    ret = ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &info);
    log_debug("%s:info(id %d, type 0x%x, name %s), ret %d", __func__, info.id,
        info.type, info.name, ret);
    if (ret) {
        ret = errno != EINVAL ? -errno : 0;
        log_debug("%s:error %s", __func__, strerror(errno));
        break;
    }

    int media_type = info.type & MEDIA_ENT_TYPE_MASK;
    if ((media_type & (MEDIA_ENT_T_DEVNODE | MEDIA_ENT_T_V4L2_SUBDEV))
        && (strstr(info.name, "USB Camera"))){
        /* Try to get the device name via udev */
        #ifdef HAVE_LIBUDEV
        ret = cam_get_devname_udev(udev, &info);
        if (ret) {
        #endif
          /* Fall back to get the device name via sysfs */
          ret = cam_get_devname_sysfs(&info);
        #ifdef HAVE_LIBUDEV
        }
        #endif
        if (!ret) {
            log_debug("get video node: %s", vdevname_buffer);
            #ifdef HAVE_LIBUDEV
            udev_unref(udev);
            udev = NULL;
            #endif
            close(fd);
            return true;
        }
    }
}

#ifdef HAVE_LIBUDEV
    if (NULL != udev)
        udev_unref(udev);
#endif
  close(fd);
  return false;
}

char *
cam_src_initialize(const char* filepath) {
  if (false == carm_src_is_usb(filepath)) {
    cam_src_obtain_devname(filepath);
  }

  log_debug("obtain devname: %s", vdevname_buffer);

  return vdevname_buffer;
}

void
cam_src_finalize() {
  log_debug("finalize");
  return;
}

void
cam_src_start() {
  char send_buffer[32] = {0};
  strcpy(send_buffer, "streamon");
  udp_sock_send(client_sockfd, send_buffer, sizeof(send_buffer));
  log_debug("start ...");
  return;
}

void
cam_src_stop() {
  char recv_buffer[32] = {0};
  strcpy(recv_buffer, "streamoff");
  udp_sock_send(client_sockfd, recv_buffer, sizeof(recv_buffer));
  log_debug("stop ...");
  return;
}

