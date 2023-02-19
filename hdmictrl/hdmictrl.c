/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: hdmisrc: Amlsrc Implementation
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
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "common.h"
#include "log.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <ctype.h>
#include <pthread.h>
#include <signal.h>
#include "TvClientWrapper.h"
#include "CTvClientLog.h"

#define _GNU_SOURCE
#define HDMIRX_SERVER_SOCKET "/tmp/hdmi-rx0"
#define MSG_BUFFER_SIZE   30
#define MSG_NUM       100

typedef enum {
	AML_HDMI_RX_SIGNAL_STABLE,
	AML_HDMI_RX_SIGNAL_LOSS,
	AML_HDMI_RX_SIGNAL_MAX
} AML_HDMI_RX_E;

typedef struct {
	AML_HDMI_RX_E msg_id;
	char msg_buf[MSG_BUFFER_SIZE];
}msg_node_data;

typedef struct msg_node {
	 msg_node_data *data;
	 struct msg_node *next;
}msg_node_t;

typedef struct {
    int msg_queue_size;
	msg_node_t *head;
	msg_node_t *tail;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
}msg_queue_t;

typedef struct {
	struct TvClientWrapper_t *tv_client_wrapper;
	int connect_socker_fd;
	int listen_fd;
	msg_queue_t t_queue;
	pthread_t t_socket_thread;
	pthread_t t_cb_thread;
	bool b_svctx_enable;
}hdmi_rx_svc_t;

static hdmi_rx_svc_t *g_t_svctx =NULL;
static int client_sockfd = -1;
static char vdevname_buffer[32] = {0};

void msg_queue_init(msg_queue_t *t_queue)
{
    if (!t_queue) {
		printf("queue null");
		return;
	}

  pthread_mutex_init(&t_queue->mutex, NULL);
  pthread_cond_init(&t_queue->cond,NULL);
  t_queue->head = t_queue->tail = NULL;

}

void msg_queue_deinit(msg_queue_t *t_queue)
{
	if (!t_queue) {
		printf("queue null");
		return;
	}
	pthread_mutex_lock(&t_queue->mutex);

	msg_node_t *t_curr = t_queue->head;
	while (t_curr) {
		msg_node_t *next = t_curr->next;
		free(t_curr);
	    t_curr = next;
	}

	t_queue->head = t_queue->tail = NULL;
	pthread_mutex_unlock(&t_queue->mutex);

	pthread_mutex_destroy(&t_queue->mutex);
	pthread_cond_destroy(&t_queue->cond);
}

void msg_queue_send_msg(msg_queue_t *t_queue,msg_node_t *t_node)
{
	printf("enter msg_queue_send_msg");
	if (!t_queue || t_node) {
		printf("queue null");
		return;
	}

	pthread_mutex_lock(&t_queue->mutex);

	msg_node_t *node = (msg_node_t *)malloc(sizeof(msg_node_t));
	if (!node) {
		pthread_mutex_unlock(&t_queue->mutex);
		return;
	}

	node->data->msg_id = t_node->data->msg_id;
	node->next = NULL;

	if (!t_queue) {
		printf("prepare get head and tail");
		t_queue->head = t_queue->tail = node;
		printf("already get head and tail");
	} else {
		t_queue->tail->next = node;
		t_queue->tail = node;
	}
	pthread_cond_signal(&t_queue->cond);
	pthread_mutex_unlock(&t_queue->mutex);
}

bool msg_queue_receive_msg(msg_queue_t *t_queue,msg_node_t *t_node)
{
	if ((NULL == t_queue) || (NULL == t_node)) {
		printf("queue null");
		return false;
	}

//   if(t_queue->head == NULL) {
// 		printf("There is no node");
// 		return false;
// 	}
	pthread_mutex_lock(&t_queue->mutex);
    pthread_cond_wait(&t_queue->cond,&t_queue->mutex);
	msg_node_t *node = t_queue->head->next;

    t_node->data->msg_id = t_queue->head->data->msg_id;
    // t_node->data->msg_buf = t_queue->head->data->msg_buf;
	free(t_queue->head);
	t_queue->head = node;

	if (t_queue->head == NULL)
		t_queue->tail = NULL;

	pthread_mutex_unlock(&t_queue->mutex);
	return true;
}


static int WriteSysfs(const char *path, const char *cmd)
{
    int fd;
    fd = open(path, O_CREAT|O_RDWR | O_TRUNC, 0777);

    if (fd >= 0) {
        write(fd, cmd, strlen(cmd));
        close(fd);
        return 0;
    }

    return -1;
}

static int DisplayInit()
{
    WriteSysfs("/sys/class/graphics/fb0/osd_display_debug", "1");
    WriteSysfs("/sys/class/graphics/fb0/blank", "1");
    WriteSysfs("/sys/kernel/debug/dri/0/vpu/blank", "1");
    return 0;

}

static void TvEventCallback(event_type_t eventType, void *eventData)
{
	printf("enter TvEventCallback");

    if (eventType == TV_EVENT_TYPE_SIGLE_DETECT) {
        SignalDetectCallback_t *signalDetectEvent = (SignalDetectCallback_t *)(eventData);
        LOGD("%s: source: %d, signalFmt: %d, transFmt: %d, status: %d, isDVI: %d, Hdrinfo: %ud.\n", __FUNCTION__,
                                                   signalDetectEvent->SourceInput,
                                                   signalDetectEvent->SignalFmt,
                                                   signalDetectEvent->TransFmt,
                                                   signalDetectEvent->SignalStatus,
                                                   signalDetectEvent->isDviSignal,
                                                   signalDetectEvent->Hdrinfo);
       if (signalDetectEvent->SignalStatus) {
			msg_node_t t_node = {0};
			t_node.data->msg_id = TVIN_SIG_STATUS_STABLE;
			printf("prepare enter msg_queue_send_msg");
			msg_queue_send_msg(&g_t_svctx->t_queue,&t_node);
	   }

    } else if (eventType == TV_EVENT_TYPE_SOURCE_CONNECT) {
        SourceConnectCallback_t *sourceConnectEvent = (SourceConnectCallback_t *)(eventData);
        LOGD("%s: source: %d, connectStatus: %d\n", __FUNCTION__,
                  sourceConnectEvent->SourceInput, sourceConnectEvent->ConnectionState);
    } else {
        LOGD("%s: invalid event.\n", __FUNCTION__);
    }
}

int check_fd(int fd, int timeout_us) {
	printf("enter check_fd\n");
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
    //   printf("select fail %d, %d %s\n", retval, errno, strerror(errno));
      return -1;
    }
    if (fd > 0 && FD_ISSET(fd, &rfds)) {
	  printf("out check_fd\n");
      return 0;
    } else {
      return -1;
    }
  }
  return -1;
}


static void disconnect_hdmi(struct TvClientWrapper_t *t_wrapper,tv_source_input_t source)
{
	printf("enter disconnect_hdmi\n");
	StopTv(t_wrapper, source);
	WriteSysfs("/sys/class/graphics/fb0/blank", "0");
	ReleaseInstance(&t_wrapper);
}

static void connect_hdmi(struct TvClientWrapper_t *t_wrapper,tv_source_input_t source)
{
	printf("enter connect_hdmi\n");
	WriteSysfs("/sys/class/graphics/fb0/blank", "0");
	StartTv(t_wrapper, source);
	// usleep(1000*50);
}

static void *process_socket_thread(void *arg)
{
	printf("enter process_socket_thread\n");
	int r;
	char recv_buffer[32] = {0};
	tv_source_input_t e_currentsource = SOURCE_HDMI1;
	hdmi_rx_svc_t *t_svctx = (hdmi_rx_svc_t *)(arg);

	while (t_svctx->b_svctx_enable) {
		printf("prepare enter check_fd\n");
	//   if (check_fd(t_svctx->connect_socker_fd, 0) == 0) {
		r = TEMP_FAILURE_RETRY(recv(t_svctx->connect_socker_fd, recv_buffer, sizeof(recv_buffer), 0));
		printf("recv_buffer = %s \n",recv_buffer);
		printf("r = %d \n",r);
		if (r == 0) {
		  printf("connection interrupted\n");
		  continue;
		}
		if (strcmp("connect", recv_buffer) == 0) {
		  printf("prepare enter connect_hdmi\n");
		  connect_hdmi(t_svctx->tv_client_wrapper,e_currentsource);
		} else if (strcmp("disconnect", recv_buffer) == 0) {
		  printf("prepare enter disconnect_hdmi\n");
		  disconnect_hdmi(t_svctx->tv_client_wrapper,e_currentsource);
		  break;
		} else {
		  printf("[process_socket_thread] not supported message ...\n");
		  continue;
		}
	//   }
	}

	return NULL;
}

static void *process_cb_thread(void *arg)
{
	printf("enter process_cb_thread\n");
	msg_node_t t_msg = {0};
	char video_dev_name[32] = {0};
    int ret = 0;

	hdmi_rx_svc_t *t_svctx = (hdmi_rx_svc_t *)(arg);
	while (t_svctx->b_svctx_enable) {
		if (msg_queue_receive_msg(&t_svctx->t_queue,&t_msg)) {
			if (t_msg.data->msg_id == (AML_HDMI_RX_E)TVIN_SIG_STATUS_STABLE)
			ret = TEMP_FAILURE_RETRY(send(t_svctx->connect_socker_fd, t_msg.data->msg_buf, strlen(t_msg.data->msg_buf), 0));
			if (ret < 0) {
					printf("send msg fail");
			}
		}
	}
  return NULL; //add
}

static void hdmi_rx_svctx_perform_connect(hdmi_rx_svc_t *t_svctx)
{
	printf("enter hdmi_rx_svctx_perform_connect \n");
	struct sockaddr_un server_unix, client_unix;
	socklen_t client_unix_len;
	int listen_fd, connfd, size;

    if (!t_svctx) {
		printf("t_svctx null");
		return;
	}
    unlink(HDMIRX_SERVER_SOCKET);
	if ((listen_fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
	  perror("socket error");
	  exit(1);
	}
	t_svctx->listen_fd = listen_fd;
	memset(&server_unix, 0, sizeof(server_unix));
	server_unix.sun_family = AF_UNIX;
	strcpy(server_unix.sun_path, HDMIRX_SERVER_SOCKET);
	size = offsetof(struct sockaddr_un, sun_path) + strlen(server_unix.sun_path);

	if (bind(listen_fd, (struct sockaddr *)&server_unix, size) < 0) {
	  perror("bind error");
	  exit(1);
	}
	printf("UNIX domain socket bound\n");

	if (listen(listen_fd, 16) < 0) {
	  perror("listen error");
	  exit(1);
	}
	printf("Accepting connections ...\n");

	client_unix_len = sizeof(client_unix);
	printf("get the client_unix_len\n");

	if ((connfd = accept(listen_fd, (struct sockaddr *)&client_unix, &client_unix_len)) < 0) {
	  perror("accept error");
	  return;//add
	}

	printf("t_svctx prepare get the connect_socker_fd \n");
	t_svctx->connect_socker_fd = connfd;
	printf("connected_sockfd: %d\n", t_svctx->connect_socker_fd);

}

static hdmi_rx_svc_t *hdmi_rx_svctx_init()
{
	hdmi_rx_svc_t *t_svctx = (hdmi_rx_svc_t*) malloc(sizeof(hdmi_rx_svc_t));
	if (!t_svctx) {
		printf("can not malloc hdmi svctx \n");
		return NULL;
	}

    printf("prepare get the GetInstance \n");
    t_svctx->tv_client_wrapper = GetInstance();
	if (!t_svctx->tv_client_wrapper) {
		printf("Can not get tv client wrapper \n");
		return NULL;
	}

	printf("go hdmi_rx_svctx_perform_connect \n");
	hdmi_rx_svctx_perform_connect(t_svctx);
	return t_svctx;
}

static void Signalhandler(int sig){
    // StopTv(t_wrapper, source);
    // ReleaseInstance(&t_wrapper);
	printf("enter Signalhandler \n");
	unlink(HDMIRX_SERVER_SOCKET);
    exit(0);
}

int main(int argc, char **argv)
{
     WriteSysfs("/sys/class/vdin/vdin1/v4l_work_mode", "1");
	// fprintf(stderr,"enter main \n");
	printf("prepare enter main \n");
	signal(SIGINT, Signalhandler);
	int err = 0;
    printf("prepare enter setTvEventCallback \n");
    setTvEventCallback(TvEventCallback);

	printf("prepare enter DisplayInit \n");
    DisplayInit();

	printf("prepare enter hdmi_rx_svctx_init \n");
	g_t_svctx = hdmi_rx_svctx_init();

	printf("prepare enter msg_queue_init \n");
    msg_queue_init(&g_t_svctx->t_queue);
    g_t_svctx->b_svctx_enable = true;

    printf("prepare enter process_socket_thread \n");
	err = pthread_create(&g_t_svctx->t_socket_thread, NULL, &process_socket_thread, g_t_svctx);
	if (err != 0) {
	   printf("can not create process_socket_thread\n");
	  exit(1);
	}
	pthread_join(g_t_svctx->t_socket_thread, NULL);
	printf("out process_socket_thread\n");

    printf("prepare enter process_cb_thread \n");
	err = pthread_create(&g_t_svctx->t_cb_thread,NULL,&process_cb_thread,g_t_svctx);
	if (err != 0) {
		printf("can not create process_cb_thread\n");
		exit(1);
	}
	pthread_join(g_t_svctx->t_cb_thread, NULL);
	printf("out process_cb_thread\n");

    close(g_t_svctx->connect_socker_fd);
    close(g_t_svctx->listen_fd);

    printf("prepare enter msg_queue_deinit \n");
	msg_queue_deinit(&g_t_svctx->t_queue);
	unlink(HDMIRX_SERVER_SOCKET);
    return 0;
}

