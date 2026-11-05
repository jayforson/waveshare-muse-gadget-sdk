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
 * it gets nothing, so it cannot fire on Muse's own voice. There is
 * no voice barge-in: only a BOOT hold interrupts a reply, so no
 * standalone echo canceller and no voice-vs-speaker judgment exist
 * in this file. (The wake AFE's own internal AEC, cfg->aec_init,
 * belongs to WakeNet's idle pipeline and is untouched.)
 *
 * Wake tuning, v6 experimental (2026-11-05, user report: "Hi, Wall E"
 * sometimes needs several shouts): three levers, all in this file.
 * (1) Threshold: set_wakenet_threshold(..., 1, 0.50) after create.
 * The wn9 model default is about 0.60-0.62 (the model name encodes
 * its range, e.g. ..._0.608_0.615); 0.50 is a moderate step down,
 * inside the API's 0.4-0.9999 range. Lower catches more, at the
 * cost of more false triggers; 0.50 is the first step, not the
 * floor. (2) Noise suppression: ns_init = 1, retried with 0 if
 * the AFE will not create (no NS model in the model partition, or
 * the check clears it). NS is single-channel here ("MNR", se off),
 * so it is the stage that should apply; the boot log's pipeline
 * printout shows whether it really engaged. (3) Wake-path gain:
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


static void heap_log(const char *what)
{
    ESP_LOGI(TAG, "%s: free internal %u, psram %u", what,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

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
    /* Wake tuning v6: try noise suppression on first, fall back to
     * off if the AFE will not create with it (no NS model in the
     * model partition, or the config check clears it). Either way
     * the board keeps working; only the log line differs. */
    int ns_applied = -1;
    for (int attempt = 0; attempt < 2 && !s_afe_data; attempt++) {
        int ns_try = attempt == 0 ? 1 : 0;
        afe_config_t *cfg = afe_config_init("MNR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
        if (!cfg) {
            ESP_LOGW(TAG, "AFE config failed (ns=%d); retrying", ns_try);
            continue;
        }
        /* The speech-recognition pipeline: voice activity, wake word,
         * neural noise suppression (v6). No echo cancellation, no
         * blind source separation (single mic). Wake is armed only
         * when idle, when the speaker is silent, so AEC has no echo
         * to cancel; leaving it off also saves internal RAM for the
         * voice task. */
        cfg->aec_init = 0;
        cfg->vad_init = 1;
        cfg->wakenet_init = 1;
        cfg->se_init = 0;
        cfg->ns_init = ns_try;
        /* The model itself is picked in menuconfig under ESP Speech
         * Recognition and loaded from the model partition; this field is
         * only logged here, not gated on, because the documented AFE flow
         * sets no model name in code and the field may stay unset even
         * with a model loaded. If the wake word never fires on the board,
         * the menuconfig pick is the first thing to re-check. */
        ESP_LOGI(TAG, "wake model: %s", cfg->wakenet_model_name ? cfg->wakenet_model_name : "(unset at init)");
        s_afe_handle = esp_afe_handle_from_config(cfg);
        s_afe_data = s_afe_handle ? s_afe_handle->create_from_config(cfg) : NULL;
        afe_config_free(cfg);
        if (s_afe_data) {
            ns_applied = ns_try;
        } else {
            ESP_LOGW(TAG, "AFE create failed with ns=%d; retrying", ns_try);
        }
    }
    if (!s_afe_data) {
        ESP_LOGW(TAG, "AFE create failed; waking off (BOOT still works)");
        esp_srmodel_deinit(models);
        heap_log("wake init done");
        return false;
    }
    /* Wake tuning v6: lower the detection threshold from the model
     * default (~0.60-0.62) to 0.50. set_wakenet_threshold takes the
     * wakenet index (1: only model loaded) and a threshold in
     * 0.4-0.9999; it returns 1 on success, -1 on failure. A failure
     * here is not fatal: the model default stays in force and wake
     * keeps working, only less sensitive than this tune intends.
     * The Mac build is the check that this API exists under the
     * pinned esp-sr ^2.0.0; if it does not, the build fails loudly
     * here and this call is the thing to drop. */
    int thr_rc = -1;
    if (s_afe_handle && s_afe_handle->set_wakenet_threshold) {
        thr_rc = s_afe_handle->set_wakenet_threshold(s_afe_data, 1, 0.50f);
    }
    ESP_LOGI(TAG, "wake tune v6: threshold 0.50 (set rc=%d), ns=%d, wake gain x2", thr_rc, ns_applied);
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


