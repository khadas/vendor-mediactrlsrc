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

#include "mediactrl_common.h"
#include "mediactrl_log.h"

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

typedef struct {
	struct TvClientWrapper_t *tv_client_wrapper;
	int connect_socker_fd;
	int listen_fd;
	pthread_t t_socket_thread;
	bool b_svctx_enable;
}hdmi_rx_svc_t;

static hdmi_rx_svc_t *g_t_svctx =NULL;
static int client_sockfd = -1;

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

static void TvEventCallback(event_type_t eventType, void *eventData)
{
	if (eventType == TV_EVENT_TYPE_SIGLE_DETECT) {
		SignalDetectCallback_t *signalDetectEvent = (SignalDetectCallback_t *)(eventData);
		log_debug("%s: source: %d, signalFmt: %d, transFmt: %d, status: %d, isDVI: %d, Hdrinfo: %ud.\n", __FUNCTION__,
			signalDetectEvent->SourceInput,
			signalDetectEvent->SignalFmt,
			signalDetectEvent->TransFmt,
			signalDetectEvent->SignalStatus,
			signalDetectEvent->isDviSignal,
			signalDetectEvent->Hdrinfo);
		if (TVIN_SIG_STATUS_STABLE == signalDetectEvent->SignalStatus) {
			log_debug("SignalStatus is stable");
			char send_buffer[32] = {0};
			strcpy(send_buffer, "sigstable");
			log_debug("send_buffer: %s", send_buffer);
			int	ret = TEMP_FAILURE_RETRY(send(g_t_svctx->connect_socker_fd, send_buffer, strlen(send_buffer), 0));
					if (ret < 0) {
					log_debug("send sigstable fail");
					}
		}
	} else if (eventType == TV_EVENT_TYPE_SOURCE_CONNECT) {
		SourceConnectCallback_t *sourceConnectEvent = (SourceConnectCallback_t *)(eventData);
		log_debug("%s: source: %d, connectStatus: %d\n", __FUNCTION__,
		sourceConnectEvent->SourceInput, sourceConnectEvent->ConnectionState);
	} else {
		log_debug("%s: invalid event %d.\n", __FUNCTION__, eventType);
	}
}

static void *process_socket_thread(void *arg)
{
	log_debug("enter process_socket_thread\n");
	int r;
	char recv_buffer[32] = {0};
	tv_source_input_t e_currentsource ;
	char *hdmirxsrc;
	hdmirxsrc = getenv("HDMISRC");
	log_debug("hdmirxsrc is %s\n",hdmirxsrc);
	if (NULL == hdmirxsrc) {
		e_currentsource = SOURCE_HDMI1;
		free(hdmirxsrc);
		log_debug("HDMISRC is hdmi1\n");
	}
	else if (0 == memcmp("hdmi1",hdmirxsrc,5)) {
		e_currentsource = SOURCE_HDMI1;
		log_debug("HDMISRC is hdmi1\n");
	}
	else if (0 == memcmp("hdmi2",hdmirxsrc,5)) {
		e_currentsource = SOURCE_HDMI2;
		log_debug("HDMISRC is hdmi2\n");
	}
	else if (0 == memcmp("hdmi3",hdmirxsrc,5)) {
		e_currentsource = SOURCE_HDMI3;
		log_debug("HDMISRC is hdmi3\n");
	}
	else{
		log_debug("invalid HDMISRC \n");
	}
	log_debug("e_currentsource = %d \n",e_currentsource);
	hdmi_rx_svc_t *g_t_svctx = (hdmi_rx_svc_t *)(arg);

	while (g_t_svctx->b_svctx_enable) {
		r = TEMP_FAILURE_RETRY(recv(g_t_svctx->connect_socker_fd, recv_buffer, sizeof(recv_buffer), 0));
		log_debug("recv_buffer = %s \n",recv_buffer);
		log_debug("r = %d \n",r);
		if (r == 0) {
		  log_debug("connection interrupted\n");
		  continue;
		}
		if (strcmp("connect", recv_buffer) == 0) {
		  log_debug("prepare enter connect_hdmi\n");
		  StartTv(g_t_svctx->tv_client_wrapper, e_currentsource);
		  log_debug("startv is ok\n");
		} else if (strcmp("disconnect", recv_buffer) == 0) {
		  log_debug("prepare enter disconnect_hdmi\n");
		  StopTv(g_t_svctx->tv_client_wrapper, e_currentsource);
		  log_debug("stoptv is ok\n");
		  ReleaseInstance(&g_t_svctx->tv_client_wrapper);
		  break;
		} else {
		  log_debug("[process_socket_thread] not supported message ...\n");
		  continue;
		}
	}
	return NULL;
}

static void hdmi_rx_svctx_perform_connect(hdmi_rx_svc_t *g_t_svctx)
{
	log_debug("enter hdmi_rx_svctx_perform_connect \n");
	struct sockaddr_un server_unix, client_unix;
	socklen_t client_unix_len;
	int listen_fd, connfd, size;

	if (!g_t_svctx) {
		log_debug("g_t_svctx null");
		return;
	}
	unlink(HDMIRX_SERVER_SOCKET);
	if ((listen_fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
	  perror("socket error");
	  exit(1);
	}
	g_t_svctx->listen_fd = listen_fd;
	memset(&server_unix, 0, sizeof(server_unix));
	server_unix.sun_family = AF_UNIX;
	strcpy(server_unix.sun_path, HDMIRX_SERVER_SOCKET);
	size = offsetof(struct sockaddr_un, sun_path) + strlen(server_unix.sun_path);

	if (bind(listen_fd, (struct sockaddr *)&server_unix, size) < 0) {
	  perror("bind error");
	  exit(1);
	}
	log_debug("UNIX domain socket (%s) bound\n", server_unix.sun_path);

	if (listen(listen_fd, 16) < 0) {
	  perror("listen error");
	  exit(1);
	}

	client_unix_len = sizeof(client_unix);
	log_debug("Accepting connections,get the client_unix_len: %d\n",client_unix_len);

	if ((connfd = accept(listen_fd, (struct sockaddr *)&client_unix, &client_unix_len)) < 0) {
	  perror("accept error");
	  return;//add
	}

	log_debug("g_t_svctx prepare get the connect_socker_fd %d\n", connfd);
	g_t_svctx->connect_socker_fd = connfd;
}

static hdmi_rx_svc_t *hdmi_rx_svctx_init()
{
	hdmi_rx_svc_t *g_t_svctx = (hdmi_rx_svc_t*) malloc(sizeof(hdmi_rx_svc_t));
	if (!g_t_svctx) {
		log_debug("malloc hdmi svctx failed\n");
		return NULL;
	}

	log_debug("prepare get the GetInstance \n");
	g_t_svctx->tv_client_wrapper = GetInstance();
	if (!g_t_svctx->tv_client_wrapper) {
		log_debug("get tv client wrapper failed\n");
		return NULL;
	}

	log_debug("go hdmi_rx_svctx_perform_connect \n");
	hdmi_rx_svctx_perform_connect(g_t_svctx);
	return g_t_svctx;
}

static void Signalhandler(int sig){
	log_debug("enter hdmictrl Signalhandler: %d\n",sig);
	unlink(HDMIRX_SERVER_SOCKET);
	exit(0);
}

int main(int argc, char **argv)
{
	log_debug("enter main,prepare enter hdmi_rx_svctx_init \n");
	g_t_svctx = hdmi_rx_svctx_init();
	signal(SIGINT, Signalhandler);

	log_debug("prepare enter setTvEventCallback \n");
	setTvEventCallback(TvEventCallback);

	log_debug("prepare enter process_socket_thread \n");
	int err = 0;
	g_t_svctx->b_svctx_enable = true;
	err = pthread_create(&g_t_svctx->t_socket_thread, NULL, &process_socket_thread, g_t_svctx);
	if (err != 0) {
		log_debug("create process_socket_thread failed %d\n", err);
		exit(1);
	}
	pthread_join(g_t_svctx->t_socket_thread, NULL);
	log_debug("out process_socket_thread\n");

	close(g_t_svctx->connect_socker_fd);
	close(g_t_svctx->listen_fd);

	unlink(HDMIRX_SERVER_SOCKET);
	return 0;
}

