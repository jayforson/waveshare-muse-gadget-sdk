/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/*
 * Local wake word (ESP-SR WakeNet through the AFE) for the voice task.
 * All calls run on the voice task only; there is no locking.
 *
 * One ESP-SR handle, created once at boot when CONFIG_MUSE_WAKE: the
 * AFE (input "MNR": mic, unused, playback reference; speech
 * recognition type, low-cost mode) feeds idle mic audio and reports
 * WakeNet detections. It needs the wake word model in the "model"
 * flash partition, picked in menuconfig under ESP Speech Recognition
 * (Jarvis first). With no model picked, waking never fires and the
 * voice task still works from BOOT. There is no voice barge-in: only
 * a BOOT hold interrupts a reply, so this file has no echo canceller
 * and no voice-vs-speaker judgment.
 *
 * When CONFIG_MUSE_WAKE is off (every board but the s185b overlay), the
 * inline fallbacks below keep the voice task compiling with waking
 * disabled.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"

#if CONFIG_MUSE_WAKE

/* Creates the handle. True when the wake-word AFE is ready. Never
 * fails the boot: on failure it logs why and waking never fires. */
bool muse_wake_init(void);

/* Feeds idle mic audio (16 kHz mono) to the wake-word AFE. True the
 * moment the wake word is detected. False when the AFE is not ready. */
bool muse_wake_feed(const int16_t *pcm, size_t frames);

#else

static inline bool muse_wake_init(void)
{
    return false;
}

static inline bool muse_wake_feed(const int16_t *pcm, size_t frames)
{
    (void)pcm;
    (void)frames;
    return false;
}

#endif
