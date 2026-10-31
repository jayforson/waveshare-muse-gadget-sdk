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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * User settings, persisted in NVS. Setters save immediately and then notify the
 * change listener (from the caller's task), which applies the setting to the
 * hardware. Brightness, auto-sleep and the speaker are polled where they're used.
 */

#define MUSE_SSID_MAX 32
#define MUSE_PASS_MAX 64
#define MUSE_HOST_MAX 63
#define MUSE_VM_MAX 63
#define MUSE_TOKEN_MAX 1023

#define MUSE_MIC_GAIN_MAX 36      /* dB; ES7210 PGA, applied in 3 dB steps */
#define MUSE_SW_GAIN_MAX 4        /* software gain multiplier, 1..4 */
#define MUSE_SILENCE_S_MIN 3      /* no-speech hangup window, seconds */
#define MUSE_SILENCE_S_MAX 30
#define MUSE_SILENCE_S_DEFAULT 5

typedef enum {
    MUSE_SETTING_VOLUME,
    MUSE_SETTING_SPEAKER,
    MUSE_SETTING_MIC_GAIN,
    MUSE_SETTING_SW_GAIN,
    MUSE_SETTING_BRIGHTNESS,
    MUSE_SETTING_SLEEP,
    MUSE_SETTING_WIFI,          /* on/off or credentials */
    MUSE_SETTING_BLE,
    MUSE_SETTING_HATCH,
    MUSE_SETTING_SPEAK_ALL,
    MUSE_SETTING_WAKE_BATTERY,
} muse_setting_t;

/* Last speech (TTS) failure, for diagnosis without a serial log. Saved in
 * NVS; set by the chat session, shown in Settings. No listener notification:
 * it is diagnostic, not a setting the hardware applies. Cleared by the next
 * successful speech fetch. */
#define TTS_FAIL_NONE 0
#define TTS_FAIL_FETCH 1      /* ElevenLabs fetch failed (network/timeout) */
#define TTS_FAIL_STATUS 2     /* ElevenLabs answered non-200; see tts_fail_status */
#define TTS_FAIL_INCOMPLETE 3 /* fetch ended without a whole MP3 */
#define TTS_FAIL_QUEUE 4      /* TTS job could not be queued (worker wedged) */
#define TTS_FAIL_STUCK 5      /* watchdog dropped a job with no progress */
#define TTS_FAIL_SEND 6       /* output buffer never drained, job aborted */

typedef void (*muse_setting_cb_t)(muse_setting_t what);

esp_err_t muse_settings_init(void);
void muse_settings_set_listener(muse_setting_cb_t cb);

int muse_settings_volume(void);         /* 0..100 */
bool muse_settings_speaker_on(void);    /* off: replies are shown, not played */
bool muse_settings_speak_all(void);     /* on: every push is spoken; off (default): only
                                        * current-conversation content is spoken, automatic
                                        * pushes are dropped before any TTS or decode */
bool muse_settings_scribe_stt(void);    /* on (default): a voice note is transcribed by
                                        * ElevenLabs Scribe first (Chinese hint), and the
                                        * text is sent as a voice turn; off: the note is
                                        * streamed to Muse as before, server transcribes */
int muse_settings_tts_fail_reason(void);  /* TTS_FAIL_* above, 0 = none */
int muse_settings_tts_fail_status(void);  /* HTTP status for TTS_FAIL_STATUS, else 0 */
void muse_settings_tts_fail_text(char *buf, size_t cap);  /* "—" when none */
bool muse_settings_wake_on_battery(void);  /* on: Jarvis wake arms even when the board reports battery */
int muse_settings_mic_gain(void);       /* dB, 0..MUSE_MIC_GAIN_MAX */
int muse_settings_sw_gain(void);        /* multiplier, 1..MUSE_SW_GAIN_MAX */
int muse_settings_silence_s(void);      /* no-speech hangup window, seconds */
int muse_settings_brightness(void);     /* 10..100 */
int muse_settings_sleep_s(void);        /* 0 = never */
bool muse_settings_wifi_on(void);
bool muse_settings_ble_on(void);

void muse_settings_wifi(char ssid[MUSE_SSID_MAX + 1], char pass[MUSE_PASS_MAX + 1]);
void muse_settings_hatch_host(char out[MUSE_HOST_MAX + 1]);
void muse_settings_hatch_vm(char out[MUSE_VM_MAX + 1]);
void muse_settings_hatch_token(char out[MUSE_TOKEN_MAX + 1]);
size_t muse_settings_hatch_token_len(void);

void muse_settings_set_volume(int pct);
void muse_settings_set_speaker_on(bool on);
void muse_settings_set_speak_all(bool on);
void muse_settings_set_scribe_stt(bool on);  /* no listener notify: the session reads it live */
void muse_settings_set_tts_fail(int reason, int status);  /* diagnostic only, no notify */
void muse_settings_set_wake_on_battery(bool on);
void muse_settings_set_mic_gain(int db);
void muse_settings_set_sw_gain(int mul);
void muse_settings_set_silence_s(int secs);
void muse_settings_set_brightness(int pct);
void muse_settings_set_sleep_s(int secs);
void muse_settings_set_wifi_on(bool on);
void muse_settings_set_ble_on(bool on);
/* A network name is remembered first among the saved ones and joined now;
 * an empty ssid forgets every saved network. */
void muse_settings_set_wifi(const char *ssid, const char *pass);
void muse_settings_set_hatch_host(const char *host);
void muse_settings_set_hatch_vm(const char *vm);
/* append=true adds to the stored token (for chunked BLE writes). */
esp_err_t muse_settings_set_hatch_token(const char *token, bool append);
