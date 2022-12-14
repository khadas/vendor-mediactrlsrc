/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Amlv4l2src Library
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

#include <stdio.h>
#include <dlfcn.h>
#include "amlsrc.h"

typedef void (*Func) (aml_src_t *);

aml_src_t amlsrc;

int aml_v4l2src_get_method(aml_src_t *pamlsrc, const char* devtype) {
  if (pamlsrc->initialize &&
      pamlsrc->finalize &&
      pamlsrc->start &&
      pamlsrc->stop) {
    return 0;
  }

  char func_name[32];
  sprintf(func_name, "aml_src_get_%s_method", devtype);
  void *handle;
  handle = dlopen("/usr/lib/libamlsrc.so", RTLD_LAZY);
  if (handle) {
    printf("func_name: %s\n", func_name);
    Func func = (Func)dlsym(handle, func_name);
    if (!func) {
      printf("find function error\n");
      dlclose(handle);
      return -1;
    }
    func(pamlsrc);
    printf("initialize func addr: %p\n", pamlsrc->initialize);
    printf("finalize func addr: %p\n", pamlsrc->finalize);
    printf("start func addr: %p\n", pamlsrc->start);
    printf("stop func addr: %p\n", pamlsrc->stop);
  }

  return 0;
}

char *aml_v4l2src_connect(const char* devname, const char* devtype) {
  if (0 == aml_v4l2src_get_method(&amlsrc, devtype)) {
    return amlsrc.initialize(devname);
  } else {
    return NULL;
  }
}


void aml_v4l2src_disconnect() {
  amlsrc.finalize();
}


void aml_v4l2src_streamon() {
  amlsrc.start();
}


void aml_v4l2src_streamoff() {
  amlsrc.stop();
}
