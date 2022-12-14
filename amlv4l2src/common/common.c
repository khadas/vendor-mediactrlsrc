/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Common Function
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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "log.h"

int udp_sock_create(const char* server_socket_path) {
  if (NULL == server_socket_path)
    return -1;

  int sockfd = -1;
  struct sockaddr_un server_unix;

  if ((sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
    log_error("client socket error");
    return -1;
  }

  memset(&server_unix, 0, sizeof(server_unix));
  server_unix.sun_family = AF_UNIX;
  strcpy(server_unix.sun_path, server_socket_path);
  int len = offsetof(struct sockaddr_un, sun_path) + strlen(server_unix.sun_path);
  if (connect(sockfd, (struct sockaddr *)&server_unix, len) < 0) {
    log_error("connect ...");
    return -1;
  }

  return sockfd;
}

void udp_sock_send(int sockfd, char *buffer, int len) {
  if (NULL == buffer)
    return;

  int r = TEMP_FAILURE_RETRY(send(sockfd, buffer, len, 0));
  if (r < 0) {
    log_debug("send failed");
    return;
  }
}

void udp_sock_recv(int sockfd, char *buffer, int len) {
  if (NULL == buffer)
    return;

  int r = TEMP_FAILURE_RETRY(recv(sockfd, buffer, len, 0));
  if (r < 0) {
    log_debug("recv failed");
    return;
  }
}
