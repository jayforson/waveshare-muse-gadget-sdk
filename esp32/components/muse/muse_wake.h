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
 * Local wake word (ESP-SR WakeNet through the AFE) and voice barge-in
 * (ESP-SR echo cancellation) for the voice task. All calls run on the
 * voice task only; there is no locking.
 *
 * Two ESP-SR handles, both created once at boot when CONFIG_MUSE_WAKE:
 * - the AFE (input "MNR": mic, unused, playback reference; speech
 *   recognition type, low-cost mode) feeds idle mic audio and reports
 *   WakeNet detections. It needs the wake word model in the "model"
 *   flash partition, picked in menuconfig under ESP Speech Recognition
 *   (Jarvis first). With no model picked, waking never fires and the
 *   voice task still works from BOOT.
 * - the AFE echo canceller (same "MNR" layout, filter length 4, speech
 *   recognition type, low-cost mode) cleans mic audio against the
 *   playback reference while a reply plays, so speech over the reply
 *   can barge in. It needs no model.
 *
 * When CONFIG_MUSE_WAKE is off (every board but the s185b overlay), the
 * inline fallbacks below keep the voice task compiling with waking and
 * barge-in disabled.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"

#if CONFIG_MUSE_WAKE

/* Creates both handles. True when the wake-word AFE is ready; echo
 * cancellation may still be ready when this is false (see
 * muse_wake_aec_frame_size). Never fails the boot: on failure it logs
 * why and waking never fires. */
bool muse_wake_init(void);

/* Feeds idle mic audio (16 kHz mono) to the wake-word AFE. True the
 * moment the wake word is detected. False when the AFE is not ready. */
bool muse_wake_feed(const int16_t *pcm, size_t frames);

/* Echo-canceller frame size in samples (0 when it is not ready). The
 * caller buffers mic and playback-reference audio to this size. */
size_t muse_wake_aec_frame_size(void);

/* Feeds mic and playback-reference audio (16 kHz mono, any chunk size)
 * to the echo canceller. Completed cleaned frames land in `cleaned`
 * (capacity `cleaned_cap` samples); `*cleaned_frames` says how many.
 * Returns false when the canceller is not ready or nothing completed. */
bool muse_wake_aec_feed(const int16_t *mic, const int16_t *ref, size_t frames,
                        int16_t *cleaned, size_t cleaned_cap, size_t *cleaned_frames);

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

static inline size_t muse_wake_aec_frame_size(void)
{
    return 0;
}

static inline bool muse_wake_aec_feed(const int16_t *mic, const int16_t *ref, size_t frames,
                                      int16_t *cleaned, size_t cleaned_cap, size_t *cleaned_frames)
{
    (void)mic;
    (void)ref;
    (void)frames;
    (void)cleaned;
    (void)cleaned_cap;
    *cleaned_frames = 0;
    return false;
}

#endif
