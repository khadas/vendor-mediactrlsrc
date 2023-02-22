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

#include "mediactrl_common.h"
#include "mediactrl_log.h"

static char *server_socket = DEFAULT_SERVER_SOCKET0;
static int client_sockfd = -1;
static char vdevname_buffer[32] = {0};

static void
cam_src_obtain_devname(const char *filepath) {
  if (strstr(filepath, "media0")) {
    server_socket = DEFAULT_SERVER_SOCKET0;
  } else if (strstr(filepath, "media1")) {
    server_socket = DEFAULT_SERVER_SOCKET1;
  } else {
    log_debug("not supported media device name ...");
    return;
  }

  if (!access(server_socket, F_OK)) {
    unlink(server_socket);
  }

  pid_t pid;
  pid = fork();
  if (pid < 0) {
    log_debug("fork error");
    exit(1);
  }
  if (pid == 0) {
    prctl(PR_SET_PDEATHSIG, SIGKILL);
    /* call execl to startup camctrl */
    execl("/usr/bin/camctrl", "camctrl", "-m", filepath, "-c", "2", NULL);
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

char *
cam_src_initialize(const char* filepath) {
  cam_src_obtain_devname(filepath);

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

