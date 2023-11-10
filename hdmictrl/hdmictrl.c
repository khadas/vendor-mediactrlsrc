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
#include <time.h>
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
    bool b_tv_released;
    pthread_mutex_t tvclient_mutex;
    bool b_tv_fake_stable;
}hdmi_rx_svc_t;

tv_source_input_t e_currentsource ;
static hdmi_rx_svc_t *g_t_svctx =NULL;
static int client_sockfd = -1;

static void _send_signal_stable()
{
    char send_buffer[32] = {0};
    strcpy(send_buffer, "sigstable");
    log_debug("send_buffer: %s, errno %d [%d]", send_buffer, errno, ECONNRESET);
    if (errno != ECONNRESET || g_t_svctx->b_tv_fake_stable == false) {
    int ret = TEMP_FAILURE_RETRY(send(g_t_svctx->connect_socker_fd, send_buffer, strlen(send_buffer), 0));
    if (ret < 0) {
        log_debug("send sigstable fail");
    }
    g_t_svctx->b_tv_fake_stable = true;
    }
}

static  void _hdmi_rc_svctx_release()
{
    if (!g_t_svctx) {
        log_debug("g_t_svctx null");
        return;
    }

    if (!g_t_svctx->b_tv_released) {
        StopTv(g_t_svctx->tv_client_wrapper, e_currentsource);
        log_debug("stoptv is ok");
        ReleaseInstance(&g_t_svctx->tv_client_wrapper);
        log_debug("ReleaseInstance is ok");
    }

    g_t_svctx->b_tv_released = true;
    close(g_t_svctx->connect_socker_fd);
    close(g_t_svctx->listen_fd);
    unlink(HDMIRX_SERVER_SOCKET);
    g_t_svctx->b_svctx_enable = false;

    log_debug("exit _hdmi_rc_svctx_release");
}

static void TvEventCallback(event_type_t eventType, void *eventData)
{
    if (eventType == TV_EVENT_TYPE_SIGLE_DETECT) {
        SignalDetectCallback_t *signalDetectEvent = (SignalDetectCallback_t *)(eventData);
        log_debug("source: %d, signalFmt: %d, transFmt: %d, status: %d, isDVI: %d, Hdrinfo: %ud",
            signalDetectEvent->SourceInput,
            signalDetectEvent->SignalFmt,
            signalDetectEvent->TransFmt,
            signalDetectEvent->SignalStatus,
            signalDetectEvent->isDviSignal,
            signalDetectEvent->Hdrinfo);
        if (TVIN_SIG_STATUS_STABLE == signalDetectEvent->SignalStatus) {
            log_debug("SignalStatus is stable");
            _send_signal_stable();
        }
    } else if (eventType == TV_EVENT_TYPE_SOURCE_CONNECT) {
        SourceConnectCallback_t *sourceConnectEvent = (SourceConnectCallback_t *)(eventData);
        log_debug("source: %d, connectStatus: %d",
        sourceConnectEvent->SourceInput, sourceConnectEvent->ConnectionState);
    } else {
        log_debug("invalid event %d.", eventType);
    }
}

static void *process_socket_thread(void *arg)
{
    log_debug("enter");
    int r;
    char recv_buffer[32] = {0};
    char *hdmirxsrc;
    hdmirxsrc = getenv("HDMISRC");
    log_debug("hdmirxsrc is %s",hdmirxsrc);
    if (NULL == hdmirxsrc) {
        e_currentsource = SOURCE_HDMI1;
        free(hdmirxsrc);
        log_debug("HDMISRC is hdmi1");
    }
    else if (0 == memcmp("hdmi1",hdmirxsrc,5)) {
        e_currentsource = SOURCE_HDMI1;
        log_debug("HDMISRC is hdmi1");
    }
    else if (0 == memcmp("hdmi2",hdmirxsrc,5)) {
        e_currentsource = SOURCE_HDMI2;
        log_debug("HDMISRC is hdmi2");
    }
    else if (0 == memcmp("hdmi3",hdmirxsrc,5)) {
        e_currentsource = SOURCE_HDMI3;
        log_debug("HDMISRC is hdmi3");
    }
    else{
        log_debug("invalid HDMISRC");
    }
    log_debug("e_currentsource = %d",e_currentsource);
    hdmi_rx_svc_t *g_t_svctx = (hdmi_rx_svc_t *)(arg);

    while (g_t_svctx->b_svctx_enable) {
        log_debug("loop wait recv cmd");
        r = TEMP_FAILURE_RETRY(recv(g_t_svctx->connect_socker_fd, recv_buffer, sizeof(recv_buffer), 0));
        log_debug("recv_buffer = %s ",recv_buffer);
        log_debug("r = %d errno = %d",r, errno);

        if (!g_t_svctx->b_svctx_enable) {
            log_debug("exit thread");
            break;
        }

        if ((r == 0) || (r == -1L) || (strcmp("disconnect", recv_buffer) == 0)) {
            _hdmi_rc_svctx_release();
            break;
        } else if (strcmp("connect", recv_buffer) == 0) {
            log_debug("Enter connect_hdmi");
            pthread_mutex_lock(&g_t_svctx->tvclient_mutex);
            StartTv(g_t_svctx->tv_client_wrapper, e_currentsource);
            log_debug("Exit connect_hdmi");
            g_t_svctx->b_tv_released = false;
            pthread_mutex_unlock(&g_t_svctx->tvclient_mutex);
            log_debug("startv is ok");
        } else {
            log_debug("[%s] not supported message ...", __FUNCTION__);
            continue;
        }
    }

    log_debug("exit");
    return NULL;
}

static void hdmi_rx_svctx_perform_connect(hdmi_rx_svc_t *g_t_svctx)
{
    log_debug("enter");
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
    struct timespec currentTime;
    clock_gettime(CLOCK_REALTIME, &currentTime);

    log_debug("[%ld]UNIX domain socket (%s) bound", currentTime.tv_nsec, server_unix.sun_path);

    if (listen(listen_fd, 16) < 0) {
      perror("listen error");
      exit(1);
    }

    client_unix_len = sizeof(client_unix);
    clock_gettime(CLOCK_REALTIME, &currentTime);
    log_debug("[%ld]Accepting connections,get the client_unix_len: %d",currentTime.tv_nsec, client_unix_len);

    if ((connfd = accept(listen_fd, (struct sockaddr *)&client_unix, &client_unix_len)) < 0) {
      perror("accept error");
      return;//add
    }

    // show network status
    system("netstat -an | grep hdmi-rx");

    log_debug("g_t_svctx prepare get the connect_socker_fd %d", connfd);
    g_t_svctx->connect_socker_fd = connfd;

    log_debug("exit");
}

static hdmi_rx_svc_t *hdmi_rx_svctx_init()
{
    log_debug("enter");
    hdmi_rx_svc_t *g_t_svctx = (hdmi_rx_svc_t*) malloc(sizeof(hdmi_rx_svc_t));
    if (!g_t_svctx) {
        log_debug("malloc hdmi svctx failed");
        return NULL;
    }

    g_t_svctx->b_tv_fake_stable = false;
    log_debug("prepare get the GetInstance");
    g_t_svctx->tv_client_wrapper = GetInstance();
    if (!g_t_svctx->tv_client_wrapper) {
        log_debug("get tv client wrapper failed");
        return NULL;
    }

    log_debug("go hdmi_rx_svctx_perform_connect");
    hdmi_rx_svctx_perform_connect(g_t_svctx);

    g_t_svctx->b_svctx_enable = true;
    g_t_svctx->b_tv_released = false;
    pthread_mutex_init(&g_t_svctx->tvclient_mutex, NULL);

    log_debug("exit");
    return g_t_svctx;
}

static void Signalhandler(int sig)
{
    log_debug("enter hdmictrl Signalhandler: %d",sig);

    //Send Fake signal stable to unlock start() wait.
    if (g_t_svctx->b_tv_fake_stable == false) {
        _send_signal_stable();
    }

    //Wait for the client's thread to exit
    pthread_join(g_t_svctx->t_socket_thread, NULL);

    exit(0);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    signal(SIGINT, SIG_IGN); // Ignore the SIGINT signal
    signal(SIGTERM, Signalhandler);

    log_debug("enter, prepare enter hdmi_rx_svctx_init");
    g_t_svctx = hdmi_rx_svctx_init();
    if (!g_t_svctx) {
        log_debug("g_t_svctx init fail");
        return -1;
    }

    log_debug("prepare enter setTvEventCallback");
    setTvEventCallback(TvEventCallback);

    log_debug("prepare enter process_socket_thread");
    int err = 0;
    err = pthread_create(&g_t_svctx->t_socket_thread, NULL, &process_socket_thread, g_t_svctx);
    if (err != 0) {
        log_debug("create process_socket_thread failed %d", err);
        exit(1);
    }

    pthread_join(g_t_svctx->t_socket_thread, NULL);
    log_debug("exit process_socket_thread");

    if (g_t_svctx) {
       free(g_t_svctx);
    }
    log_debug("exit");
    return 0;
}

