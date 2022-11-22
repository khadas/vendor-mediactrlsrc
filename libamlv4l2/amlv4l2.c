/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Amlv4l2 Library
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
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#define DEFAULT_SERVER_SOCKET "/tmp/aml-isp.socket"

static int client_sockfd = -1;
static char vdevname_buffer[32] = {0};

static void get_valid_video_device_name(void) {
  struct  sockaddr_un server_unix;

  if ((client_sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
    perror("client socket error");
    exit(1);
  }

  memset(&server_unix, 0, sizeof(server_unix));
  server_unix.sun_family = AF_UNIX;
  strcpy(server_unix.sun_path, DEFAULT_SERVER_SOCKET);
  int len = offsetof(struct sockaddr_un, sun_path) + strlen(server_unix.sun_path);
  if (connect(client_sockfd, (struct sockaddr *)&server_unix, len) < 0) {
    perror("connect error");
    exit(1);
  }

  int r = TEMP_FAILURE_RETRY(recv(client_sockfd, vdevname_buffer, sizeof(vdevname_buffer), 0));
  if (r < 0) {
    printf("recv video device name failed\n");
    return;
  }
}

char *amlv4l2_obtain_devname(const char *pathname) {
  if (!access(DEFAULT_SERVER_SOCKET, F_OK)) {
    unlink(DEFAULT_SERVER_SOCKET);
  }

  pid_t pid;
  pid = fork();
  if (pid < 0) {
    printf("fork error\n");
    exit(1);
  }
  if (pid == 0) {
    prctl(PR_SET_PDEATHSIG, SIGKILL);
    /* call execl to startup aml-isp-demo */
    execl("/usr/bin/mediactrlsrc", "mediactrlsrc", "-m", pathname, NULL);
  }

  while (true) {
    if (!access(DEFAULT_SERVER_SOCKET, F_OK))
      break;
    else
      continue;
  }

  get_valid_video_device_name();

  return vdevname_buffer;
}

void amlv4l2_close() {
  if (client_sockfd)
    close(client_sockfd);
}

void amlv4l2_notify_streamon() {
  if (client_sockfd) {
    char send_buffer[32] = {0};
    strcpy(send_buffer, "streamon");
    int r = TEMP_FAILURE_RETRY(send(client_sockfd, &send_buffer, sizeof(send_buffer), 0));
    if (r < 0) {
        printf("send streamon notification failed\n");
    }
  }
}

void amlv4l2_notify_streamoff() {
  if (client_sockfd) {
    char send_buffer[32] = {0};
    strcpy(send_buffer, "streamoff");
    int r = TEMP_FAILURE_RETRY(send(client_sockfd, &send_buffer, sizeof(send_buffer), 0));
    if (r < 0) {
        printf("send streamoff notification failed\n");
    }
  }
}

