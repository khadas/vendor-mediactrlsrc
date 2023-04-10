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
#define hdmictrl_SERVER_SOCKET "/tmp/hdmi-rx0"
#define MSG_BUFFER_SIZE   30
#define MSG_NUM       100

static char *server_socket = hdmictrl_SERVER_SOCKET;
static int client_sockfd = -1;

static void
hdmi_src_obtain_devname(const char *filepath) {

    if (!access(server_socket, F_OK)) {
    unlink(server_socket);
  }
  log_debug("enter hdmi_src_obtain_devname \n");
  pid_t pid;
  pid = fork();
  if (pid < 0) {
    log_debug("fork error");
    exit(1);
  }
  log_debug("hdmi_src_obtain_devname, pid=%d \n", pid);
  if (pid == 0) {
    prctl(PR_SET_PDEATHSIG, SIGKILL);
    /* call execl to startup hdmienable */
    log_debug("ready to execl /usr/bin/hdmictrl, pid=%d \n", pid);
    execl("/usr/bin/hdmictrl", "hdmictrl", NULL);
    log_debug("execl /usr/bin/hdmictrl ok \n");
  }
  log_debug("hdmi_src_obtain_devname end, pid=%d \n", pid);

  while (true) {
    if (!access(server_socket, F_OK))
      break;
    else
      continue;
  }

  client_sockfd = udp_sock_create(server_socket);//need add
}

char *
hdmi_src_initialize(const char* filepath) {
  log_debug("enter  hdmi_src_initialize \n");

  hdmi_src_obtain_devname(filepath);
  char send_buffer[32] = {0};
  strcpy(send_buffer, "connect");
  udp_sock_send(client_sockfd, send_buffer, sizeof(send_buffer));
  log_debug("send_buffer: %s", send_buffer);

  return NULL;
}

void
hdmi_src_finalize() {
  log_debug("finalize\n");

  char send_buffer[32] = {0};
  strcpy(send_buffer, "disconnect");
  udp_sock_send(client_sockfd, send_buffer, sizeof(send_buffer));
  log_debug("send_buffer: %s", send_buffer);
  return;
}

void
hdmi_src_start() {
  log_debug("enter  hdmi_src_start \n");

  char recv_buffer[32] = {0};
  udp_sock_recv(client_sockfd, recv_buffer, sizeof(recv_buffer));
  log_debug("recv_buffer: %s", recv_buffer);
  return;
}

void
hdmi_src_stop() {
  log_debug("enter  hdmi_src_stop \n");
  log_debug("stop ...\n");
  return;
}



