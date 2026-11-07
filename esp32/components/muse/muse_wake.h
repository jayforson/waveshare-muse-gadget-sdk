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
 * voice task still works from BOOT.
 *
 * Voice barge-in (v6 trial, AEC+VAD): the same AFE is created with
 * AEC on when it fits, and while a reply plays the voice task feeds
 * mic + just-played PCM (reference) through muse_wake_bargein_feed;
 * the AFE's VAD then reports speech on the echo-cancelled signal.
 * If AEC does not fit (creation fails, or too little internal RAM
 * is left for the call-time Scribe TLS), the AFE is recreated
 * without AEC, barge-in is off, and waking works as before.
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

/* Voice barge-in: true when the AFE was created with AEC and is
 * ready to judge speech during playback. False when AEC did not
 * fit (barge-in off, waking unaffected). */
bool muse_wake_bargein_ready(void);

/* Feeds one paired chunk to the AFE during playback: mic audio and
 * the PCM just played (reference), same length, 16 kHz mono.
 * Returns the AFE's VAD state for the latest processed frame:
 * true when it reports speech on the echo-cancelled signal, false
 * when it reports silence. The state is sticky across calls: a
 * feed that does not complete a frame leaves it unchanged, so the
 * caller can count wall-clock speech instead of completed frames
 * (a 320-sample feed completes a frame only every ~1.6 calls, and
 * treating the in-between calls as silence resets the count every
 * other call, so it can never reach the threshold). The caller
 * counts sustained speech and decides when to interrupt. */
bool muse_wake_bargein_feed(const int16_t *mic, const int16_t *ref, size_t frames);

/* Clears the barge-in feed state for a new reply: the partial frame
 * and the sticky VAD, so a previous reply's tail cannot trigger the
 * next one. */
void muse_wake_bargein_reset(void);

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

static inline bool muse_wake_bargein_ready(void)
{
    return false;
}

static inline bool muse_wake_bargein_feed(const int16_t *mic, const int16_t *ref, size_t frames)
{
    (void)mic;
    (void)ref;
    (void)frames;
    return false;
}

static inline void muse_wake_bargein_reset(void)
{
}

#endif
