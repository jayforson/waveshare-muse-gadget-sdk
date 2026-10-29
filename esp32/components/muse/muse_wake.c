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


/* Measurement (v3.1): log free AND largest for every heap. Free alone
 * lies: free big + largest small means fragmentation (space exists but
 * no block fits), free and largest both small means true exhaustion.
 * DMA is a subset view of internal RAM, not an additional heap. */
static void heap_log(const char *what)
{
    ESP_LOGI(TAG, "%s: free/largest internal %u/%u, psram %u/%u, dma %u/%u", what,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
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
    afe_config_t *cfg = afe_config_init("MNR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!cfg) {
        ESP_LOGW(TAG, "AFE config failed; waking off (BOOT still works)");
        esp_srmodel_deinit(models);
        heap_log("wake init done");
        return false;
    }
    /* The speech-recognition pipeline: voice activity, wake word. No
     * echo cancellation, no blind source separation (single mic) and
     * no neural noise suppression. Wake is armed only when idle, when
     * the speaker is silent, so AEC has no echo to cancel; leaving
     * it off also saves internal RAM for the voice task. */
    cfg->aec_init = 0;
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
    s_afe_data = s_afe_handle ? s_afe_handle->create_from_config(cfg) : NULL;
    afe_config_free(cfg);
    if (!s_afe_data) {
        ESP_LOGW(TAG, "AFE create failed; waking off (BOOT still works)");
        esp_srmodel_deinit(models);
        heap_log("wake init done");
        return false;
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
        memcpy(s_feed_acc + s_feed_acc_len, pcm + in, take * sizeof(int16_t));
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


