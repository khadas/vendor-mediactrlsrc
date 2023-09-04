/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Mediactrlsrc
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

#define _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <dlfcn.h>
#include "mediactrl_common.h"
#include "mediactrl_log.h"

#include "mediactl.h"
#include "v4l2subdev.h"
#include "v4l2videodev.h"
#include "mediaApi.h"
#include "lens_config.h"

#include "staticPipe.h"
#include "ispMgr.h"

// #define PRINT_FPS

static int connected_sockfd = -1;

static char *media_device_name = NULL;
static char *server_socket = DEFAULT_SERVER_SOCKET0;
static struct media_device *media_dev = NULL;
static int camera_num = 1;

/* media stream */
media_stream_t v4l2_media_stream;

//#define WDR_ENABLE
#define V4L2_META_AML_ISP_CONFIG    v4l2_fourcc('A', 'C', 'F', 'G') /* Aml isp param config */
#define V4L2_META_AML_ISP_STATS     v4l2_fourcc('A', 'S', 'T', 'S') /* Aml isp statistics */

#define DEFAULT_BUFFER_NUM 4

typedef struct v4l2buffer_info {
  void *pstart;
  int length;
} v4l2buffer_info_t;

enum MEDIACTRLSRC_STREAM {
  MEDIACTRLSRC_STREAM_OUTPUT0 = 0,    /* Not Used */
  MEDIACTRLSRC_STREAM_RAW,            /* Not Used */
  MEDIACTRLSRC_STREAM_STATISTICS,
  MEDIACTRLSRC_STREAM_MAX
};
pthread_t tid[MEDIACTRLSRC_STREAM_MAX];

struct thread_info {
  pthread_t p_tid;
  uint32_t status;

  /* sem_t p_sem; */

  /* aml isp algorithm related */
  aisp_calib_info_t calib;
  AML_ALG_CTX_S pstAlgCtx;
};

struct thread_param {
  //v4l2buffer_info_t *buffer_isp_output0[DEFAULT_BUFFER_NUM];  /* reserved */
  v4l2buffer_info_t buffer_isp_stats[DEFAULT_BUFFER_NUM];
  v4l2buffer_info_t buffer_isp_param;

  struct sensorConfig *sensor_cfg;
  struct lensConfig   *lens_cfg;

  /* format related */
  uint32_t width;
  uint32_t height;
  // uint32_t pixformat;
  uint32_t fmt_code;
  uint32_t wdr_mode;


  uint32_t capture_count;
  int fps;

  pthread_mutex_t mutex;
  pthread_cond_t cond;
  pthread_t process_socket_tid;
  bool streaming;

  struct thread_info info;
};

/*
 * definition of tparam (struct thread_param)
 * set default value
 */
static struct thread_param tparam = {
  .width = 3840,
  .height = 1920,
  // .pixformat = V4L2_PIX_FMT_NV12, /* V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_Y12, V4L2_PIX_FMT_SRGGB12 */
#ifdef WDR_ENABLE
  .fmt_code = MEDIA_BUS_FMT_SBGGR10_1X10, /* MEDIA_BUS_FMT_SRGGB12_1X12 */
  .wdr_mode = WDR_MODE_2To1_FRAME, /* WDR_MODE_2To1_LINE */
#else
  .fmt_code = MEDIA_BUS_FMT_SBGGR10_1X10, /* MEDIA_BUS_FMT_SRGGB12_1X12 */
  .wdr_mode = WDR_MODE_NONE,
#endif

  .capture_count = 0,
  .fps = 0,

  .mutex = PTHREAD_MUTEX_INITIALIZER,
  .cond = PTHREAD_COND_INITIALIZER,
  .streaming = false,
};

/*
 * check video device capability
 */
static int check_capability(struct media_entity *entity) {
  struct v4l2_capability v4l2_cap;
  memset (&v4l2_cap, 0, sizeof (struct v4l2_capability));
  int ret = v4l2_video_get_capability(entity, &v4l2_cap);
  if (ret < 0) {
    log_error("get entity[%s] -> video[%s] capability failed",
      entity->info.name,
      entity->devname);
    return -1;
  }

  log_debug("entity[%s] -> video[%s], cap.driver:%s, capabilities:0x%x, device_caps:0x%x",
    entity->info.name,
    entity->devname,
    v4l2_cap.driver,
    v4l2_cap.capabilities,
    v4l2_cap.device_caps);

  return 0;
}

static int media_set_wdr_mode(media_stream_t *stream, uint32_t wdr_mode) {
  int ret = -1;
  media_set_wdrMode(stream, 0);
  ret = media_set_wdrMode(stream, wdr_mode);

  return ret;
#if 0
  /* sensor wdr mode */
  if (wdr_mode != ISP_SDR_DCAM_MODE) {
    ret = v4l2_subdev_set_wdr(stream->sensor.entity, wdr_mode);
    if (ret < 0) {
      log_error("set sensor wdr mode failed");
      return ret;
    }
  }

  /* isp-adapter wdr mode */
  ret = v4l2_subdev_set_wdr(stream->isp_adapter.entity, wdr_mode);
  if (ret < 0) {
    log_error("set isp-adapter wdr mode failed");
    return ret;
  }

  /* isp-core wdr mode */
  ret = v4l2_subdev_set_wdr(stream->isp_core.entity, wdr_mode);
  if (ret < 0) {
    log_error("set isp-core wdr mode failed");
    return ret;
  }

  log_debug("set wdr mode successfully");
  return ret;
#endif
}

static int media_stream_init(media_stream_t *stream,
    struct media_device *media_dev) {

  mediaStreamInit(stream, media_dev);

  android::staticPipe::fetchPipeMaxResolution(
      &v4l2_media_stream,
      tparam.width,
      tparam.height
  );

  if (1 == camera_num) {
    tparam.fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
    tparam.wdr_mode = WDR_MODE_NONE;
  } else if (2 == camera_num) {
    tparam.fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
    tparam.wdr_mode = ISP_SDR_DCAM_MODE;
  } else {
    log_error("only support 1 or 2 camera");
    return 0;
  }

  /* set wdr mode */
  media_set_wdr_mode(stream, tparam.wdr_mode);

  /* TODO */

  return 0;
}


/*
 * set subdev(entity pad) format
 * include sensor, isp-csiphy, isp-adapter, isp-core
 */
static int set_subdev_pad_format(media_stream_t *stream,
  stream_configuration_t *cfg) {
  int ret = -1;
  struct v4l2_mbus_framefmt mbus_format;
  enum v4l2_subdev_format_whence which = V4L2_SUBDEV_FORMAT_ACTIVE;

  mbus_format.width  = cfg->format.width;
  mbus_format.height = cfg->format.height;
  mbus_format.code   = cfg->format.code;

  /* sensor source pad[0] format */
  ret = v4l2_subdev_set_format(stream->sensor_ent,
    &mbus_format, 0, which);
  if (ret < 0) {
    log_error("set subdev sensor pad[0] format failed");
    return ret;
  }

  /* isp-csiphy sink & source pad[0, 1] format */
  ret = v4l2_subdev_set_format(stream->csiphy_ent,
    &mbus_format, 0, which);
  if (ret < 0) {
    log_error("set subdev isp-ciphy pad[0] format failed");
    return ret;
  }
  ret = v4l2_subdev_set_format(stream->csiphy_ent,
    &mbus_format, 1, which);
  if (ret < 0) {
    log_error("set subdev isp-csiphy pad[1] format failed");
    return ret;
  }

  /* isp-adapter sink & source pad[0, 1] format */
  ret = v4l2_subdev_set_format(stream->adap_ent,
    &mbus_format, 0, which);
  if (ret < 0) {
    log_error("set subdev isp-adapter pad[0] format failed");
    return ret;
  }
  ret = v4l2_subdev_set_format(stream->adap_ent,
    &mbus_format, 1, which);
  if (ret < 0) {
    log_error("set subdev isp-adapter pad[1] format failed");
    return ret;
  }

  /* isp-core sink pad[0] format */
  ret = v4l2_subdev_set_format(stream->isp_ent,
    &mbus_format, 0, which);
  if (ret < 0) {
    log_error("set subdev isp-core pad[0] format failed");
    return ret;
  }

  return ret;
}

/*
 * link and activate subdev(entity pad) which
 * includes sensor, isp-csiphy, isp-adapter, isp-core
 */
#define SENSOR_SOURCE_PAD_INDEX 0
#define SINK_PAD_INDEX 0
#define SOURCE_PAD_INDEX 1
static int link_and_activate_subdev(media_stream_t *stream) {
  int ret = -1;

  struct media_pad *sink_pad = NULL;
  struct media_pad *source_pad = NULL;
  int flag = MEDIA_LNK_FL_ENABLED;    /* The link is enabled and can be used to transfer media data */

  /* source pad[0] sensor --> sink pad[0] isp-csiphy */
  source_pad = (struct media_pad*)media_entity_get_pad(stream->sensor_ent, SENSOR_SOURCE_PAD_INDEX);
  if (!source_pad) {
    log_error("get sensor source pad[0] failed");
    return -1;
  }
  sink_pad = (struct media_pad*)media_entity_get_pad(stream->csiphy_ent, SINK_PAD_INDEX);
  if (!sink_pad) {
    log_error("get isp-csiphy sink pad[0] failed");
    return -1;
  }
  ret = media_setup_link(media_dev, source_pad, sink_pad, flag);
  if (ret != 0) {
    log_error("link sensor to isp-csiphy failed");
    return ret;
  }

  /* source pad[1] isp-csiphy --> sink pad[0] isp-adapter */
  source_pad = (struct media_pad*)media_entity_get_pad(stream->csiphy_ent, SOURCE_PAD_INDEX);
  if (!source_pad) {
    log_error("get isp-csiphy source pad[1] failed");
    return -1;
  }
  sink_pad = (struct media_pad*)media_entity_get_pad(stream->adap_ent, SINK_PAD_INDEX);
  if (!sink_pad) {
    log_error("get isp-adapter sink pad[0] failed");
    return -1;
  }
  ret = media_setup_link(media_dev, source_pad, sink_pad, flag);
  if (ret != 0) {
    log_error("link isp-csiphy to isp-adapter failed");
    return ret;
  }

  /* source pad[1] isp-adapter --> sink pad[0] isp-core */
  source_pad = (struct media_pad*)media_entity_get_pad(stream->adap_ent, SOURCE_PAD_INDEX);
  if (!source_pad) {
    log_error("get isp-adapter source pad[1] failed");
    return -1;
  }
  sink_pad = (struct media_pad*)media_entity_get_pad(stream->isp_ent, SINK_PAD_INDEX);
  if (!sink_pad) {
    log_error("get isp-core sink pad[0] failed");
    return -1;
  }
  ret = media_setup_link(media_dev, source_pad, sink_pad, flag);
  if (ret != 0) {
    log_error("link isp-adapter to isp-core failed");
    return ret;
  }

  log_debug("link and activate subdev successfully");

  return ret;
}

/*
 * media stream config
 * >1. setup necessary subdev's pad format
 * >2. link and activate necessary subdev
 */
int media_stream_config(media_stream_t *stream) {
  int ret = -1;

  stream_configuration_t stream_cfg;
  memset(&stream_cfg, 0, sizeof(stream_configuration_t));
  stream_cfg.format.width = tparam.width;
  stream_cfg.format.height = tparam.height;
  // stream_cfg.format.fourcc = tparam.pixformat;
  stream_cfg.format.code = tparam.fmt_code;;
  stream_cfg.format.nplanes = 1;

  ret = set_subdev_pad_format(stream, &stream_cfg);
  if (ret < 0) {
    log_error("set subdev format failed");
    return ret;
  }

  ret = link_and_activate_subdev(stream);
  if (ret < 0) {
    log_error("link and activate subdev failed");
    return ret;
  }

  log_debug("config media stream successfully");

  return ret;
}



/*
 * set isp-stats entity format (out: statistics)
 */
static int set_isp_stats_fmt(media_stream_t *stream,
  stream_configuration_t *cfg) {
  int ret = -1;
  struct v4l2_format v4l2_fmt;

  memset (&v4l2_fmt, 0, sizeof (struct v4l2_format));

  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_fmt.fmt.pix_mp.width = cfg->format.width;
  v4l2_fmt.fmt.pix_mp.height = cfg->format.height;
  v4l2_fmt.fmt.pix_mp.pixelformat = V4L2_META_AML_ISP_STATS;
  v4l2_fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;

  ret = v4l2_video_set_format(stream->video_stats, &v4l2_fmt);
  if (ret < 0) {
    log_error("set isp-stats fmt failed, ret:%d", ret);
    return ret;
  }

  return 0;
}

/*
 * set isp-param entity format (in: parameter)
 */
static int set_isp_param_fmt(media_stream_t *stream,
  stream_configuration_t *cfg) {
  int ret = -1;
  struct v4l2_format v4l2_fmt;

  memset (&v4l2_fmt, 0, sizeof (struct v4l2_format));

  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_fmt.fmt.pix_mp.width = cfg->format.width;
  v4l2_fmt.fmt.pix_mp.height = cfg->format.height;
  v4l2_fmt.fmt.pix_mp.pixelformat = V4L2_META_AML_ISP_CONFIG;
  v4l2_fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;

  ret = v4l2_video_set_format(stream->video_param, &v4l2_fmt);
  if (ret < 0) {
    log_error("set isp-param fmt failed, ret:%d", ret);
    return ret;
  }

  return 0;
}

/* isp algorithm interface */
struct ispIF isp_alg_iface;
static int get_isp_alg_interface() {
  void *handle;
  handle = dlopen("/usr/lib/libispaml.so", RTLD_NOW);
  if (!handle) {
    printf("dlopen libispaml.so error: %s\n", dlerror());
    return -1;
  }

  isp_alg_iface.alg2User = (isp_alg2user)dlsym(handle, "aisp_alg2user");
  isp_alg_iface.alg2Kernel = (isp_alg2kernel)dlsym(handle, "aisp_alg2kernel");
  isp_alg_iface.algEnable = (isp_enable)dlsym(handle, "aisp_enable");
  isp_alg_iface.algDisable = (isp_disable)dlsym(handle, "aisp_disable");
  isp_alg_iface.algFwInterface = (isp_fw_interface)dlsym(handle, "aisp_fw_interface");


  printf("alg2User func addr: %p\n", isp_alg_iface.alg2User);
  printf("alg2Kernel func addr: %p\n", isp_alg_iface.alg2Kernel);
  printf("algEnable func addr: %p\n", isp_alg_iface.algEnable);
  printf("algDisable func addr: %p\n", isp_alg_iface.algDisable);
  printf("algFwInterface func addr: %p\n", isp_alg_iface.algFwInterface);

  return 0;
}

/*
 * init isp algorithm thread related parameter
 * >1. set isp-stats and isp-param entity format
 * >2. request and map buffer respectively
 * >3. first queue to clean buffer before streamon
 * >4. streamon
 */
static void isp_alg_param_init() {
  int ret = -1;

  check_capability(v4l2_media_stream.video_stats);
  check_capability(v4l2_media_stream.video_param);

  stream_configuration_t common_cfg;
  common_cfg.format.width = 1024;
  common_cfg.format.height = 256;
  common_cfg.format.nplanes = 1;
  set_isp_stats_fmt(&v4l2_media_stream, &common_cfg);
  set_isp_param_fmt(&v4l2_media_stream, &common_cfg);

  /* isp-stats & isp-param request buffers */
  struct v4l2_requestbuffers v4l2_rb;
  memset (&v4l2_rb, 0, sizeof (struct v4l2_requestbuffers));
  v4l2_rb.count = DEFAULT_BUFFER_NUM;
  v4l2_rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_rb.memory = V4L2_MEMORY_MMAP;
  ret = v4l2_video_req_bufs(v4l2_media_stream.video_stats, &v4l2_rb);
  if (ret < 0) {
    log_error("isp-stats request buffer error");
    return;
  }
  memset (&v4l2_rb, 0, sizeof (struct v4l2_requestbuffers));
  v4l2_rb.count = 1;
  v4l2_rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_rb.memory = V4L2_MEMORY_MMAP;
  ret = v4l2_video_req_bufs(v4l2_media_stream.video_param, &v4l2_rb);
  if (ret < 0) {
    log_error("isp-param request buffer error");
    return;
  }


  /*  map isp-stats & isp-param buffers */
  struct v4l2_buffer v4l2_buf;
  for (int i = 0; i < DEFAULT_BUFFER_NUM; i++) {
    memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
    v4l2_buf.index = i;
    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_buf.memory = V4L2_MEMORY_MMAP;
    ret = v4l2_video_query_buf(v4l2_media_stream.video_stats, &v4l2_buf);
    if (ret < 0) {
      log_error("isp-stats query buffer error, ret: %d", ret);
      return;
    }
    if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
      tparam.buffer_isp_stats[i].length = v4l2_buf.length;
      log_debug("isp stats query buffer, length: %u, offset: %d",
        v4l2_buf.length, v4l2_buf.m.offset);
      tparam.buffer_isp_stats[i].pstart = mmap (NULL, v4l2_buf.length,
        PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_media_stream.video_stats->fd, v4l2_buf.m.offset);
      if (tparam.buffer_isp_stats[i].pstart == MAP_FAILED) {
        log_error("isp stats map buffer error");
        return;
      }
    }
  }

  memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
  v4l2_buf.index = 0;
  v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_buf.memory = V4L2_MEMORY_MMAP;
  ret = v4l2_video_query_buf(v4l2_media_stream.video_param, &v4l2_buf);
  if (ret < 0) {
    log_error("isp-param query buffer error, ret: %d", ret);
    return;
  }
  if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
    tparam.buffer_isp_param.length = v4l2_buf.length;
    log_debug("isp param query buffer, length: %u, offset: %d",
      v4l2_buf.length, v4l2_buf.m.offset);
    tparam.buffer_isp_param.pstart = mmap (NULL, v4l2_buf.length,
      PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_media_stream.video_param->fd, v4l2_buf.m.offset);
    if (tparam.buffer_isp_param.pstart == MAP_FAILED) {
      log_error("isp stats map buffer error");
      return;
    }
  }

  /* TODO:
   * add interface of aml isp algorithm
   */
  /* Initialize algorithm related */
  char alg_init[256 * 1024];

  if (-1 == get_isp_alg_interface()) {
    log_error("get isp algorithm interface failed");
    return;
  }

  tparam.sensor_cfg = matchSensorConfig(&v4l2_media_stream);
  if (NULL == tparam.sensor_cfg) {
    log_error("match sensor config failed");
    return;
  }
  tparam.lens_cfg = matchLensConfig(&v4l2_media_stream);
  if (tparam.lens_cfg != NULL) {
      lens_set_entity(tparam.lens_cfg, v4l2_media_stream.lens_ent);
      lens_control_cb(tparam.lens_cfg, &tparam.info.pstAlgCtx.stLensFunc);
  }
#ifdef  WDR_ENABLE
  cmos_set_sensor_entity(tparam.sensor_cfg, v4l2_media_stream.sensor_ent, 1);
#else
  cmos_set_sensor_entity(tparam.sensor_cfg, v4l2_media_stream.sensor_ent, 0);
#endif

  cmos_sensor_control_cb(tparam.sensor_cfg, &tparam.info.pstAlgCtx.stSnsExp);
  cmos_get_sensor_calibration(tparam.sensor_cfg, v4l2_media_stream.sensor_ent, &tparam.info.calib);


  (isp_alg_iface.algEnable)(0, &tparam.info.pstAlgCtx, &tparam.info.calib);
  memset(alg_init, 0, sizeof(alg_init));
  (isp_alg_iface.alg2User)(0, alg_init);
  (isp_alg_iface.alg2Kernel)(0, tparam.buffer_isp_param.pstart);

  /* first queue isp-stats & isp-param buffers to clean */
  for (int i = 0; i < DEFAULT_BUFFER_NUM; i++) {
    memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
    v4l2_buf.index = i;
    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_buf.memory  = V4L2_MEMORY_MMAP;
    ret = v4l2_video_q_buf(v4l2_media_stream.video_stats, &v4l2_buf);
    if (ret < 0) {
      log_error("isp-stats first queue buffer error, ret: %d, index:  %d", ret, i);
      return;
    }
  }
  memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
  v4l2_buf.index = 0;
  v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_buf.memory = V4L2_MEMORY_MMAP;
  ret = v4l2_video_q_buf(v4l2_media_stream.video_param, &v4l2_buf);
  if (ret < 0) {
    log_error("isp-param first queue buffer error, ret: %d", ret);
    return;
  }

  /* streamon isp-stats & isp-param */
  ret = v4l2_video_stream_on(v4l2_media_stream.video_stats,
    V4L2_BUF_TYPE_VIDEO_CAPTURE);
  if (ret < 0) {
    log_error("isp-stats streamon error, ret: %d", ret);
    return;
  }
  ret = v4l2_video_stream_on(v4l2_media_stream.video_param,
    V4L2_BUF_TYPE_VIDEO_CAPTURE);
  if (ret < 0) {
    log_error("isp-stats streamon error, ret: %d", ret);
    return;
  }

  log_debug("Finish initializing amlgorithm parameter ...");
}

#ifdef PRINT_FPS
static int64_t get_current_time_msec(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#endif


/**
 * check fd whether is readable
 **/
int check_fd(int fd, int timeout_us) {
  if (timeout_us >= 0) {
    struct timeval tv;
    struct timeval *ptv;
    if (timeout_us > 0) {
      tv.tv_sec = timeout_us / 1000000;
      tv.tv_usec = timeout_us % 1000000;
      ptv = &tv;
    } else {
      ptv = NULL;
    }
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    int retval = select(fd + 1, &rfds, NULL, NULL, ptv);
    if (retval <= 0) {
      log_error("select fail %d, %d %s", retval, errno, strerror(errno));
      return -1;
    }
    if (fd > 0 && FD_ISSET(fd, &rfds)) {
      return 0;
    } else {
      return -1;
    }
  }
  return -1;
}


/**
 * isp algorithm process one
 **/
static void isp_alg_process_one(struct thread_param *tparam) {
  struct v4l2_buffer v4l2_buf_video_statistics;
  struct v4l2_buffer v4l2_buf_video_param;
  int ret = -1;
  int idx = -1;

  memset (&v4l2_buf_video_statistics, 0, sizeof (struct v4l2_buffer));
  v4l2_buf_video_statistics.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_buf_video_statistics.memory = V4L2_MEMORY_MMAP;
  ret = v4l2_video_dq_buf(
    v4l2_media_stream.video_stats,
    &v4l2_buf_video_statistics
  );
  if (ret < 0) {
    log_error("dequeue video statistics buffer failed");
    return;
  }

  memset (&v4l2_buf_video_param, 0, sizeof (struct v4l2_buffer));
  v4l2_buf_video_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_buf_video_param.memory = V4L2_MEMORY_MMAP;
  ret = v4l2_video_dq_buf(
    v4l2_media_stream.video_param,
    &v4l2_buf_video_param
  );
  if (ret < 0) {
    log_error("dequeue video param buffer failed");
    v4l2_video_q_buf(
      v4l2_media_stream.video_stats,
      &v4l2_buf_video_statistics
    );
    return;
  }

  idx = v4l2_buf_video_statistics.index;

  (isp_alg_iface.alg2User)(0, tparam->buffer_isp_stats[idx].pstart);
  (isp_alg_iface.alg2Kernel)(0, tparam->buffer_isp_param.pstart);

  /* TODO
   * ask what for lately
   */
  usleep(1000*5);

  ret = v4l2_video_q_buf(
    v4l2_media_stream.video_stats,
    &v4l2_buf_video_statistics
  );
  if (ret < 0) {
    log_error("queue video statistics buffer failed");
    return;
  }
  ret = v4l2_video_q_buf(
    v4l2_media_stream.video_param,
    &v4l2_buf_video_param
  );
  if (ret < 0) {
    log_error("queue video param buffer failed");
    return;
  }

  log_debug("process one frame successfully");
  return;
}


/**
 * isp algorithm thread
 **/
static void *isp_alg_thread(void *arg) {
  struct thread_param *tparam = (struct thread_param *)arg;

#if 0
  int stream_type = -1;
  pthreadt tid_self = pthread_self();

  for (i = 0; i < MEDIACTRLSRC_STREAM_MAX; i++) {
    if (tid_self == tid[i]) {
      stream_type = i;
      break;
    }
  }
#endif

  /**
   * use isp algorithm to process
   **/
  pthread_mutex_lock(&tparam->mutex);
  if (!tparam->streaming) {
    pthread_cond_wait(&tparam->cond, &tparam->mutex);
  }
  pthread_mutex_unlock(&tparam->mutex);

#ifdef PRINT_FPS
  uint64_t frame_count = 0;
  int64_t start, end;
  start = get_current_time_msec();
#endif

  int vf_cnt = 0;
  while (true) {
    pthread_mutex_lock(&tparam->mutex);
    if (tparam->streaming) {
      pthread_mutex_unlock(&tparam->mutex);
      vf_cnt++;
      if (vf_cnt % 2 == 0) {
        isp_alg_process_one(tparam);
      }
#ifdef PRINT_FPS
      frame_count++;
      /* print fps info every 100 frames */
      if ((frame_count % 100 == 0)) {
        end = get_current_time_msec();
        log_debug("fps: %ld",
          (100 * 1000) / (end - start));
      }
      start = get_current_time_msec();
#endif
    } else {
      pthread_mutex_unlock(&tparam->mutex);
      log_debug("streamoff ...");
      break;
    }
  }

  /* streamoff and unmap isp-stats & isp-param buffers */
  v4l2_video_stream_off(
    v4l2_media_stream.video_stats,
    V4L2_BUF_TYPE_VIDEO_CAPTURE
  );
  v4l2_video_stream_off(
    v4l2_media_stream.video_param,
    V4L2_BUF_TYPE_VIDEO_CAPTURE
  );

  for (int i = 0; i < DEFAULT_BUFFER_NUM; i++) {
    munmap(tparam->buffer_isp_stats[i].pstart,
      tparam->buffer_isp_stats[i].length);
  }
  munmap(tparam->buffer_isp_param.pstart,
    tparam->buffer_isp_param.length);

  log_debug("exit");
  return NULL;
}

static void *process_socket_thread(void *arg) {
  struct thread_param *tparam = (struct thread_param *)arg;
  int r;
  char recv_buffer[32] = {0};

  while (true) {
    if (check_fd(connected_sockfd, 0) == 0) {
      r = TEMP_FAILURE_RETRY(recv(connected_sockfd, recv_buffer, sizeof(recv_buffer), 0));
      if (r == 0) {
        log_debug("connection interrupted");
        continue;
      }
      if (strcmp("streamon", recv_buffer) == 0) {
        pthread_mutex_lock(&tparam->mutex);
        tparam->streaming = true;
        pthread_cond_signal(&tparam->cond);
        pthread_mutex_unlock(&tparam->mutex);
        log_debug("receive streamon notification");
      } else if (strcmp("streamoff", recv_buffer) == 0) {
        pthread_mutex_lock(&tparam->mutex);
        tparam->streaming = false;
        pthread_cond_signal(&tparam->cond);
        pthread_mutex_unlock(&tparam->mutex);
        log_debug("receive streamoff notification");
        break;
      } else {
        log_warn("not supported message ...");
        continue;
      }
    }
  }

  return NULL;
}

static void parse_opt(int argc, char *argv[]) {
  int opt;
  while ((opt = getopt(argc, argv, "m:c:t:s:")) != -1) {
    switch (opt) {
      case 'm': /* media device name (/dev/media0) */
        media_device_name = optarg;
        // if (strstr(media_device_name, "media0")) {
        //   server_socket = DEFAULT_SERVER_SOCKET0;
        // } else if (strstr(media_device_name, "media1")) {
        //   server_socket = DEFAULT_SERVER_SOCKET1;
        // }
        log_debug("media device name: %s", media_device_name);
        break;
      case 'c': /* camera number (0,1) */
        camera_num = atoi(optarg);
        break;
      case 't':
        break;
      case 's':
        server_socket = optarg;
        log_debug("Server socket: %s",server_socket);
        break;
      default:
        exit(1);
    }
  }
}

static void Signalhandler(int sig)
{
  char streamoff_buffer[32] = {0};
  strcpy(streamoff_buffer, "streamoff_done");
  int r = TEMP_FAILURE_RETRY(send(connected_sockfd, streamoff_buffer, strlen(streamoff_buffer), 0));
  if (r < 0) {
    log_error("send streamoff_buffer, failed");
  }
  log_debug("enter camctrl Signalhandler: %d",sig);
  unlink(server_socket);
  close(connected_sockfd);
  log_debug("exit camctrl Signalhandler: %d",sig);
  exit(0);
}

int main(int argc, char *argv[]) {

  signal(SIGINT, Signalhandler);
  signal(SIGTERM, Signalhandler);

  parse_opt(argc, argv);

  /**
   * media ctrl progress:
   *  open media device and enumerate entities (In general: /dev/media0)
   *  get video device name through entity name
   *
   **/
  media_dev = media_device_new(media_device_name);
  if (NULL == media_dev) {
    log_debug("new media device error");
    return -1;
  }
  if (0 != media_device_enumerate(media_dev)) {
    log_error("media device enumerate failed");
    return -1;
  }
  media_stream_init(&v4l2_media_stream, media_dev);
  media_stream_config(&v4l2_media_stream);
  isp_alg_param_init();

  /**
   * Unix Socket Operation:
   *  [Progress] send video device name
   *
   **/
  struct sockaddr_un server_unix, client_unix;
  socklen_t client_unix_len;
  int listen_fd, connfd, size;

  if ((listen_fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
    log_error("socket error");
    exit(1);
  }

  memset(&server_unix, 0, sizeof(server_unix));
  server_unix.sun_family = AF_UNIX;
  strcpy(server_unix.sun_path, server_socket);
  size = offsetof(struct sockaddr_un, sun_path) + strlen(server_unix.sun_path);
  unlink(server_socket);
  if (bind(listen_fd, (struct sockaddr *)&server_unix, size) < 0) {
    log_error("bind error");
    exit(1);
  }
  log_debug("UNIX domain socket bound");

  if (listen(listen_fd, 16) < 0) {
    log_error("listen error");
    exit(1);
  }
  log_debug("Accepting connections ...");

  client_unix_len = sizeof(client_unix);
  if ((connfd = accept(listen_fd, (struct sockaddr *)&client_unix, &client_unix_len)) < 0) {
    log_error("accept error");
    return -1;
  }
  connected_sockfd = connfd;
  log_debug("connected_sockfd: %d", connected_sockfd);

  char video_dev_name[32] = {0};
  strcpy(video_dev_name, v4l2_media_stream.video_ent0->devname);
  log_debug("video_dev_name: %s", video_dev_name);
  int r = TEMP_FAILURE_RETRY(send(connected_sockfd, video_dev_name, strlen(video_dev_name), 0));
  if (r < 0) {
    log_error("send video_dev_name, failed");
  }

  int err;
  err = pthread_create(&tparam.process_socket_tid, NULL, &process_socket_thread, &tparam);
  if (err != 0) {
    log_error("can't create process_socket_thread");
    exit(1);
  }
  err = pthread_create(&tid[MEDIACTRLSRC_STREAM_STATISTICS], NULL, &isp_alg_thread, &tparam);
  if (err != 0) {
    log_error("can't create isp_alg_thread");
    exit(1);
  }

  pthread_join(tparam.process_socket_tid, NULL);
  pthread_join(tid[MEDIACTRLSRC_STREAM_STATISTICS], NULL);

  char streamoff_buffer[32] = {0};
  strcpy(streamoff_buffer, "streamoff_done");
  int m = TEMP_FAILURE_RETRY(send(connected_sockfd, streamoff_buffer, strlen(streamoff_buffer), 0));
  if (m < 0) {
    log_error("send streamoff_buffer, failed");
  }
  unlink(server_socket);
  close(connected_sockfd);
  close(listen_fd);
  log_debug("camctrl exit");
  return 0;
}
