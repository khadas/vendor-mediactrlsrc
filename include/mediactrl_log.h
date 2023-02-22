/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Log Function
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

#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>


static inline char *get_cur_time() {
    static char s[20];
    time_t t;
    struct tm* ltime;
    time(&t);
    ltime = localtime(&t);
    strftime(s, 20, "%Y-%m-%d %H:%M:%S", ltime);
    return s;
}


#ifndef LOGLEVEL
#define LOGLEVEL 4
#endif

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#if LOGLEVEL < 3
#define NDEBUG 1
#endif

#ifdef NDEBUG
#define log_debug(M, ...)
#else
#ifdef LOG_NOCOLORS
  #define log_debug(M, ...) fprintf(stderr, "[%s]  DEBUG  [amlv4l2src %s:%d:%s]" M "\n", \
    get_cur_time(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
  /* white color */
  #define log_debug(M, ...) fprintf(stderr, "[%s]  \33[1;37mDEBUG\33[0;39m  [amlv4l2src %s:%d:%s]" M "\n", \
    get_cur_time(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#endif /*  NOCOLORS */
#endif /*  NDEBUG */

/*  safe readable version of errno */
#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#ifdef LOG_NOCOLORS
  #define log_error(M, ...) fprintf(stderr, "[%s]  ERROR  [amlv4l2src %s:%d:%s] errno:%s " M "\n", \
    get_cur_time(), __FILE__, __LINE__, __func__, clean_errno(), ##__VA_ARGS__)
  #define log_warn(M, ...) fprintf(stderr, "[%s]  WARN  [amlv4l2src %s:%d:%s] errno:%s" M "\n", \
    get_cur_time(), __FILE__, __LINE__, __func__, clean_errno(), ##__VA_ARGS__)
  #define log_info(M, ...) fprintf(stderr, "[%s]  INFO  [amlv4l2src %s:%d:%s]" M "\n", \
    get_cur_time(), __FILENAME__, __LINE__, __func__, ##__VA_ARGS__)
#else
  /* red color */
  #define log_error(M, ...) fprintf(stderr, "[%s]  \33[1;31mERROR\33[0;39m  [amlv4l2src %s:%d:%s] errno:%s" M "\n", \
    get_cur_time(), __FILE__, __LINE__, __func__, clean_errno(), ##__VA_ARGS__)
  /* yellow color */
  #define log_warn(M, ...) fprintf(stderr, "[%s]  \33[1;33mWARN\33[0;39m  [amlv4l2src %s:%d:%s] errno:%s" M "\n", \
    get_cur_time(), __FILE__, __LINE__, __func__, clean_errno(), ##__VA_ARGS__)
  /* blue color */
  #define log_info(M, ...) fprintf(stderr, "[%s]  \33[1;34mINFO\33[0;39m  [amlv4l2src %s:%d:%s]" M "\n", \
    get_cur_time(), __FILENAME__, __LINE__, __func__, ##__VA_ARGS__)
#endif /*  NOCOLORS */

#if LOGLEVEL < 4
#undef log_info
#define log_info(M, ...)
#endif

#if LOGLEVEL < 2
#undef log_warn
#define log_warn(M, ...)
#endif

#if LOGLEVEL < 1
#undef log_error
#define log_error(M, ...)
#endif

#endif  /* __LOG_H__ */
