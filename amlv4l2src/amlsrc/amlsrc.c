/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in below
 * which is part of this source code package.
 *
 * Description: Amlsrc Interface
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

#include "amlsrc.h"
#include <dlfcn.h>
#include <stdio.h>

#define SRC_FUNCTION(moduletype) \
void aml_src_get_##moduletype##_method(aml_src_t *amlsrc) {                             \
  void *handle;                                                                         \
  handle = dlopen("/usr/lib/lib"#moduletype"src.so", RTLD_LAZY);                        \
  if (!handle) {                                                                        \
    printf("dlopen lib error: %s\n", dlerror());                                        \
    return;                                                                             \
  }                                                                                     \
  amlsrc->initialize = (AmlSrcInitialize)dlsym(handle, #moduletype"_src_initialize");   \
  amlsrc->finalize = (AmlSrcFinalize)dlsym(handle, #moduletype"_src_finalize");         \
  amlsrc->start = (AmlSrcStart)dlsym(handle, #moduletype"_src_start");                  \
  amlsrc->stop = (AmlSrcStop)dlsym(handle, #moduletype"_src_stop");                     \
}


SRC_FUNCTION(cam)
SRC_FUNCTION(hdmi)

