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

/*
 * Local wake word, from ESP-SR WakeNet through the AFE (see
 * muse_wake.h). Symbols used here are the ones ESP-SR's own AFE
 * documentation and its test_apps/esp-sr-afe test show: the AFE
 * interface (esp_afe_sr_iface.h, esp_afe_config.h). No standalone
 * WakeNet interface is used: in current ESP-SR, WakeNet runs inside
 * the AFE and reports through its fetch interface.
 *
 * Only the voice task calls into this file. The wake-word AFE is fed
 * only while idle (the voice task arms it); during a call or a reply
 * it gets nothing, so it cannot fire on Muse's own voice.
 *
 * Voice barge-in (v6 trial, AEC+VAD): the same AFE is created with
 * AEC on when it fits (see muse_wake_init), and while a reply plays
 * the voice task feeds mic + just-played PCM (reference) through
 * muse_wake_bargein_feed; the AFE's VAD then reports speech on the
 * echo-cancelled signal, and sustained speech interrupts the reply
 * like a BOOT hold. If AEC does not fit, the AFE is recreated
 * without it, barge-in is off, and waking works as before.
 *
 * Wake tuning, v6 experimental (2026-11-05, user report: "Hi, Wall E"
 * sometimes needs several shouts): three levers, all in this file.
 * (1) Threshold: set_wakenet_threshold(..., 1, 0.55) after create.
 * The wn9 model default is about 0.60-0.62 (the model name encodes
 * its range, e.g. ..._0.608_0.615); 0.55 is a moderate step down,
 * inside the API's 0.4-0.9999 range. Lower catches more, at the
 * cost of more false triggers. History: 0.50 (first step) caught
 * the wake word but false-triggered constantly in the evening of
 * 2026-11-05, so 0.55 is the middle ground; if it still
 * false-triggers, the next lever is the wake gain, not a lower
 * threshold. (2) Noise suppression: TRIED in v6 and REVERTED the same
 * day. ns_init = 1 created fine (pipeline showed NS(WebRTC)
 * engaged, threshold set rc=1), but the board then spammed
 * "Ringbuffer of AFE is empty" and wake stopped working entirely;
 * ESP-SR itself logs "Noise Suppression may reduce the accuracy of
 * speech recognition. It is not recommended to turn it on." The NS
 * stage starves the fetch path (fetch_with_delay(1) never sees
 * output in time), so WakeNet never sees speech. ns_init is back
 * to 0; do not re-enable without a fetch-delay rework first.
 * (3) Wake-path gain:
 * the feed copy is multiplied x2 with saturation, WakeNet only;
 * the recorded speech (pre-roll ring, Scribe) is untouched, and
 * the user's stored mic/SW gain settings are untouched. Deliberately
 * NOT changed here: AFE_MODE stays LOW_COST. HIGH_PERF costs
 * internal RAM this board does not have to spare (wake init leaves
 * ~100-200 bytes free internal; see muse_voice.c), so it is a later
 * lever only if this tune proves the pipeline but not the pickup.
 */

#include "muse_wake.h"

#include <stdlib.h>
#include <string.h>

#include "esp_afe_config.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "muse_wake";

/* Wake-word AFE (input "MNR": mic, unused, playback reference). */
static const esp_afe_sr_iface_t *s_afe_handle;
static esp_afe_sr_data_t *s_afe_data;
static int s_feed_chunksize;   /* samples per channel per AFE frame */
static int s_feed_nch;         /* channels per AFE frame (3 for "MNR") */
static int16_t *s_feed_buff;   /* one interleaved AFE frame */
static int16_t *s_feed_acc;    /* mic samples waiting for a full AFE frame */
static size_t s_feed_acc_len;
/* Voice barge-in (v6 trial): true when the AFE was created with AEC.
 * The barge-in feed has its own accumulators, separate from the idle
 * wake feed's, so the two paths never mix gains or partial frames. */
static bool s_bargein_available;
static int16_t *s_bargein_acc;      /* mic samples waiting for a full AFE frame */
static int16_t *s_bargein_ref_acc;  /* reference samples, paired with s_bargein_acc */
static size_t s_bargein_acc_len;


static void heap_log(const char *what)
{
    ESP_LOGI(TAG, "%s: free internal %u, psram %u", what,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

/* Builds the AFE with the given AEC setting, or NULL. The pipeline
 * is otherwise fixed: voice activity + wake word, no blind source
 * separation (single mic), no noise suppression. NS stays off on
 * purpose: v6 tried ns_init = 1, it created fine, but the board
 * then spammed "Ringbuffer of AFE is empty" and wake stopped
 * working (see the header comment). */
static esp_afe_sr_data_t *create_afe(srmodel_list_t *models, int aec_init)
{
    afe_config_t *cfg = afe_config_init("MNR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!cfg) {
        return NULL;
    }
    cfg->aec_init = aec_init;
    cfg->vad_init = 1;
    cfg->wakenet_init = 1;
    cfg->se_init = 0;
    cfg->ns_init = 0;
    /* The model itself is picked in menuconfig under ESP Speech
     * Recognition and loaded from the model partition; this field is
     * only logged here, not gated on, because the documented AFE flow
     * sets no model name in code and the field may stay unset even
     * with a model loaded. If the wake word never fires on the board,
     * the menuconfig pick is the first thing to re-check. */
    ESP_LOGI(TAG, "wake model: %s", cfg->wakenet_model_name ? cfg->wakenet_model_name : "(unset at init)");
    s_afe_handle = esp_afe_handle_from_config(cfg);
    esp_afe_sr_data_t *data = s_afe_handle ? s_afe_handle->create_from_config(cfg) : NULL;
    afe_config_free(cfg);
    return data;
}

/* Scribe's TLS setup needs a 6K+ largest internal block (7K works,
 * 6K fails; see the v5 heap analysis). The AEC attempt is kept only
 * if at least this much headroom remains after it. */
#define BARGEIN_MIN_LARGEST_INTERNAL 8192

bool muse_wake_init(void)
{
    /* Wake word: models come from the "model" flash partition, picked in
     * menuconfig under ESP Speech Recognition. */
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGW(TAG, "no models in the model partition; waking off (BOOT still works)");
        heap_log("wake init done");
        return false;
    }
    /* Voice barge-in (v6 trial): try the AFE with AEC first. If it
     * does not fit (creation fails, or too little internal RAM is
     * left for the call-time Scribe TLS), destroy it and recreate
     * without AEC: barge-in is off, waking works as before. */
    s_afe_data = create_afe(models, 1);
    if (s_afe_data) {
        size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        if (largest < BARGEIN_MIN_LARGEST_INTERNAL) {
            ESP_LOGW(TAG, "AEC leaves only %u largest internal block; barge-in off",
                     (unsigned)largest);
            s_afe_handle->destroy(s_afe_data);
            s_afe_data = NULL;
        } else {
            s_bargein_available = true;
            ESP_LOGI(TAG, "AEC on for barge-in (largest internal %u)", (unsigned)largest);
        }
    }
    if (!s_afe_data) {
        s_afe_data = create_afe(models, 0);
        s_bargein_available = false;
    }
    if (!s_afe_data) {
        ESP_LOGW(TAG, "AFE create failed; waking off (BOOT still works)");
        esp_srmodel_deinit(models);
        heap_log("wake init done");
        return false;
    }
    int ns_applied = 0;
    /* Wake tuning v6: lower the detection threshold from the model
     * default (~0.60-0.62) to 0.55. set_wakenet_threshold takes the
     * wakenet index (1: only model loaded) and a threshold in
     * 0.4-0.9999; it returns 1 on success, -1 on failure. A failure
     * here is not fatal: the model default stays in force and wake
     * keeps working, only less sensitive than this tune intends.
     * The Mac build is the check that this API exists under the
     * pinned esp-sr ^2.0.0; if it does not, the build fails loudly
     * here and this call is the thing to drop. */
    int thr_rc = -1;
    if (s_afe_handle && s_afe_handle->set_wakenet_threshold) {
        thr_rc = s_afe_handle->set_wakenet_threshold(s_afe_data, 1, 0.55f);
    }
    ESP_LOGI(TAG, "wake tune v6: threshold 0.55 (set rc=%d), ns=%d, wake gain x2, aec=%d (barge-in %s)",
             thr_rc, ns_applied, s_bargein_available ? 1 : 0,
             s_bargein_available ? "on" : "off");
    if (thr_rc != 1) {
        ESP_LOGW(TAG, "set_wakenet_threshold failed (rc=%d); model default threshold in force", thr_rc);
    }
    /* Log the effective pipeline, so the boot log shows whether NS
     * really engaged, e.g. [input] -> |NS| -> |VAD| ->
     * |WakeNet(wn9_hiwalle_tts2)| -> [output]. */
    if (s_afe_handle && s_afe_handle->print_pipeline) {
        s_afe_handle->print_pipeline(s_afe_data);
    }
    s_feed_chunksize = s_afe_handle->get_feed_chunksize(s_afe_data);
    s_feed_nch = s_afe_handle->get_feed_channel_num(s_afe_data);
    s_feed_buff = heap_caps_malloc((size_t)s_feed_chunksize * (size_t)s_feed_nch * sizeof(int16_t),
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_feed_acc = heap_caps_malloc((size_t)s_feed_chunksize * sizeof(int16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_feed_buff || !s_feed_acc) {
        ESP_LOGW(TAG, "AFE buffers failed; waking off (BOOT still works)");
        s_afe_handle->destroy(s_afe_data);
        s_afe_data = NULL;
        esp_srmodel_deinit(models);
        heap_log("wake init done");
        return false;
    }
    /* Barge-in accumulators (mic + reference, paired). If these fail,
     * only barge-in is off; waking keeps working. */
    s_bargein_acc = heap_caps_malloc((size_t)s_feed_chunksize * sizeof(int16_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_bargein_ref_acc = heap_caps_malloc((size_t)s_feed_chunksize * sizeof(int16_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_bargein_acc || !s_bargein_ref_acc) {
        ESP_LOGW(TAG, "barge-in buffers failed; barge-in off");
        s_bargein_available = false;
    }
    /* Models stay loaded for the AFE's lifetime (never deinitialized). */
    ESP_LOGI(TAG, "wake ready: feed %d x %d channels", s_feed_chunksize, s_feed_nch);
    heap_log("wake init done");
    return true;
}

bool muse_wake_feed(const int16_t *pcm, size_t frames)
{
    if (!s_afe_data || !s_feed_buff || !s_feed_acc || s_feed_chunksize <= 0) {
        return false;
    }
    size_t in = 0;
    while (in < frames) {
        size_t room = (size_t)s_feed_chunksize - s_feed_acc_len;
        size_t take = frames - in < room ? frames - in : room;
        /* Wake tuning v6: wake-path gain x2, WakeNet only. The input
         * chunk is shared with the pre-roll ring and the record path,
         * so the gain is applied on the copy into the AFE accumulator,
         * with saturation, never in place; recorded speech keeps the
         * user's own mic/SW gain. Louder far-field speech reaches the
         * detector; close loud speech may clip in this copy, which
         * WakeNet tolerates better than a miss at distance. */
        for (size_t i = 0; i < take; i++) {
            int x = (int)pcm[in + i] * 2;
            s_feed_acc[s_feed_acc_len + i] = x > 32767 ? 32767 : x < -32768 ? -32768 : (int16_t)x;
        }
        s_feed_acc_len += take;
        in += take;
        if (s_feed_acc_len < (size_t)s_feed_chunksize) {
            break;
        }
        /* Idle: nothing plays, so reference and unused channels are zero. */
        for (int i = 0; i < s_feed_chunksize; i++) {
            for (int ch = 0; ch < s_feed_nch; ch++) {
                s_feed_buff[i * s_feed_nch + ch] = 0;
            }
            s_feed_buff[i * s_feed_nch] = s_feed_acc[i];
        }
        s_feed_acc_len = 0;
        s_afe_handle->feed(s_afe_data, s_feed_buff);
        /* One tick of patience: the fetch only reports what the pipeline
         * has finished, and the idle loop polls again in 20 ms anyway. */
        afe_fetch_result_t *res = s_afe_handle->fetch_with_delay(s_afe_data, 1);
        if (res && res->wakeup_state == WAKENET_DETECTED) {
            return true;
        }
    }
    return false;
}

bool muse_wake_bargein_ready(void)
{
    return s_bargein_available && s_afe_data != NULL && s_bargein_acc != NULL && s_bargein_ref_acc != NULL;
}

/* Voice barge-in (v6 trial): feeds mic + just-played PCM to the
 * AEC-enabled AFE and reports the VAD state of the echo-cancelled
 * signal. The caller counts sustained speech. Only the voice task
 * calls this, only while a reply plays, so it never runs at the
 * same time as the idle wake feed; the accumulators are separate
 * anyway. No wake gain here: the caller already applied the SW
 * gain, and the AEC wants the mic as recorded. The reference is
 * the played PCM as written, unscaled; the AEC adapts to the
 * codec/volume gain. "MNR": channel 0 mic, channel 1 unused,
 * last channel playback reference. */
bool muse_wake_bargein_feed(const int16_t *mic, const int16_t *ref, size_t frames)
{
    if (!muse_wake_bargein_ready() || !s_feed_buff || s_feed_chunksize <= 0) {
        return false;
    }
    size_t in = 0;
    bool speech = false;
    while (in < frames) {
        size_t room = (size_t)s_feed_chunksize - s_bargein_acc_len;
        size_t take = frames - in < room ? frames - in : room;
        for (size_t i = 0; i < take; i++) {
            s_bargein_acc[s_bargein_acc_len + i] = mic[in + i];
            s_bargein_ref_acc[s_bargein_acc_len + i] = ref[in + i];
        }
        s_bargein_acc_len += take;
        in += take;
        if (s_bargein_acc_len < (size_t)s_feed_chunksize) {
            break;
        }
        for (int i = 0; i < s_feed_chunksize; i++) {
            for (int ch = 0; ch < s_feed_nch; ch++) {
                s_feed_buff[i * s_feed_nch + ch] = 0;
            }
            s_feed_buff[i * s_feed_nch] = s_bargein_acc[i];
            s_feed_buff[i * s_feed_nch + (s_feed_nch - 1)] = s_bargein_ref_acc[i];
        }
        s_bargein_acc_len = 0;
        s_afe_handle->feed(s_afe_data, s_feed_buff);
        /* One tick of patience, as in the wake feed. */
        afe_fetch_result_t *res = s_afe_handle->fetch_with_delay(s_afe_data, 1);
        if (res && res->vad_state == VAD_SPEECH) {
            speech = true;
        }
    }
    return speech;
}


