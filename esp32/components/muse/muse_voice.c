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

#include "muse_voice.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "muse_adpcm.h"
#include "muse_audio.h"
#include "muse_board.h"
#include "muse_chat.h"
#include "muse_input.h"
#include "muse_mem.h"
#include "muse_settings.h"
#include "muse_state.h"
#include "muse_wake.h"
#include "muse_wifi.h"

static const char *TAG = "muse_voice";

#define MAX_SECS 30
#define TAIL_FRAMES (MUSE_AUDIO_RATE * 12 / 100)   /* capture lag + poll interval, stops before the release click */
#define MAX_FRAMES (MUSE_AUDIO_RATE * MAX_SECS)
#define MIN_HELD_FRAMES (MUSE_AUDIO_RATE * 3 / 10)   /* shorter presses are taps, not speech */
#define SPEECH_MIN_CHUNKS 10   /* 0.2 s above CALL_SPEECH_DB counts as speech; isolated clicks are 1-5 chunks */
#if CONFIG_MUSE_BOARD_M5STACK_CARDPUTER_ADV
#define PRE_CHUNKS 6                                   /* 120 ms; a 320 ms upload burst stalls on the ADV */
#else
#define PRE_CHUNKS 16                                  /* 320 ms of audio kept from before the press */
#endif
#define SETTLE_CHUNKS 10   /* after Muse makes a sound, 200 ms of capture is its own tail */
#define REST_BACKSTOP_MS 60000
#define SNORE_STALE_MS 500    /* a snore asked for longer ago than this is dropped */
#define SNORE_AMP 1100.0f     /* peak; the chirp's is ~7000: about 16 dB quieter */

#if CONFIG_MUSE_HATCH
/* A note recorded while Hatch is out of reach is saved in PSRAM, and goes once it's back. */
#define HOLD_NOTES 1
#else
#define HOLD_NOTES 0   /* no PSRAM: a press needs Hatch */
#endif
#define HELD_MAX 1                            /* only the latest note is kept: a new held note drops the previous one */
#define HELD_TRIES 8                            /* then a saved note is dropped */
#define HELD_POLL_MS 5000                       /* resting with notes saved: check for Wi-Fi this often */
#define HELD_KEEP_US (30LL * 60 * 1000000)      /* muse_voice_notes_waiting() */
#define RETRY_MIN_US (15LL * 1000000)
#define RETRY_MAX_US (120LL * 1000000)
#define STALL_US (30LL * 1000000)               /* Hatch took none of a note this long: give up */
#define ACK_WAIT_US (30LL * 1000000)            /* asleep, waiting for the VM to have a note */

static QueueHandle_t s_queue;
static volatile bool s_monitor;
static volatile float s_monitor_db = -100.0f;
static volatile bool s_chirp;
static volatile bool s_loopback;
static volatile bool s_mp3test;
static volatile bool s_cancel_requested;   /* red cancel button: drop the note in progress */
static volatile uint32_t s_snore_ms;   /* when a snore was asked for (ms since boot), 0 for none */

/*
 * Wake-word calls. The voice task arms waking only while idle, plugged
 * in and within reach of Muse; idle_capture feeds the wake-word engine
 * and sets s_wake_fired. A call is the same turn machinery as a
 * push-to-talk note, looped: listen (voice activity ends the utterance),
 * send, play the reply, listen again, until 15 s of silence or a
 * hangup. s_call is also read by the input task (BOOT short press
 * during a call hangs up instead of sleeping).
 */
static bool s_call;
static volatile bool s_hangup_requested;
static bool s_reply_failed;      /* last hatch_reply ended in an error */
static char s_last_error[96];   /* error text from that failed reply */

static char s_last_reply[128];   /* final page of the last reply, for closing detection */
static char s_last_heard[96];    /* transcript of the utterance in progress */

/* True when a (short) text is a closing exchange: it ends with a
 * closing phrase and is short enough that the phrase is the point,
 * not a passing word in a long reply. "Thanks for your support" does
 * not match (it ends with "support"); a long reply ending in "thanks"
 * is filtered out by the length cap. Bare acknowledgements like a
 * lone "好的" are deliberately NOT closings: they are too ambiguous
 * (they also answer commands), so those calls still open follow-up,
 * now only for 15 s of silence instead of 30 s. */
static bool reply_is_closing(const char *text)
{
    if (!text || !text[0]) {
        return false;
    }
    size_t len = strlen(text);
    while (len > 0) {
        unsigned char c = (unsigned char)text[len - 1];
        if (c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == '.' || c == '!' || c == '?' || c == ',') {
            len--;
            continue;
        }
        /* Trailing CJK punctuation is 3 bytes (E3 80 xx, EF BC xx,
         * EF BD xx); Chinese characters start at E4, so stripping
         * these cannot eat a character. */
        if ((len >= 3 && (unsigned char)text[len - 3] == 0xE3 && (unsigned char)text[len - 2] == 0x80) ||
            (len >= 3 && (unsigned char)text[len - 3] == 0xEF && (unsigned char)text[len - 2] == 0xBC) ||
            (len >= 3 && (unsigned char)text[len - 3] == 0xEF && (unsigned char)text[len - 2] == 0xBD)) {
            len -= 3;
            continue;
        }
        break;
    }
    if (len == 0 || len > 36) {
        return false;
    }
    static const char *const CLOSINGS[] = {
        "再见", "拜拜", "晚安", "好梦", "不客气", "谢谢",
        "就这样", "就这样吧", "先这样", "先这样吧", "没事了",
        "下次见", "下次再聊", "回头见", "明天见",
        "Good night", "Goodbye", "Bye", "bye", "See you",
    };
    for (size_t i = 0; i < sizeof(CLOSINGS) / sizeof(*CLOSINGS); i++) {
        size_t cl = strlen(CLOSINGS[i]);
        if (len >= cl && memcmp(text + len - cl, CLOSINGS[i], cl) == 0) {
            return true;
        }
    }
    return false;
}
static volatile bool s_wake_armed;
static volatile bool s_wake_fired;

/* Call listening, in dBFS per 20 ms chunk. Starting points to tune on
 * the device: a quiet room sits well below these, normal speech well
 * above. Voice activity, not the cloud transcript, drives them, so a
 * loud room can hold a call open; the BOOT short press hangs up. */
#define CALL_SPEECH_DB (-45.0f)    /* speech starts above this */
#define CALL_SILENCE_DB (-50.0f)   /* an utterance ends below this */
#define CALL_GAP_CHUNKS 75         /* 1.5 s of silence ends an utterance */
#define CALL_SILENCE_US (15LL * 1000000)   /* no speech this long hangs up */

/* Call capture outcomes. */
typedef enum {
    CALL_UTTERANCE,   /* a note to send */
    CALL_CANCELLED,   /* dropped (cough, too short); the call keeps listening */
    CALL_END,         /* cancel tap: end the whole call */
    CALL_HANGUP,      /* silence timeout, hangup request, or no memory */
} call_outcome_t;

/*
 * Pre-roll: while idle the mic keeps running into this ring, so a recording
 * can start a little before the press. People start talking as they press,
 * and the power key reports the press late.
 */
typedef struct {
#if MUSE_LOW_MEM
    muse_adpcm_t start;   /* encoder state before this chunk, to decode it alone */
    uint8_t adpcm[MUSE_AUDIO_CHUNK / 2];
#else
    int16_t pcm[MUSE_AUDIO_CHUNK];
#endif
} pre_chunk_t;

static pre_chunk_t *s_pre;   /* PRE_CHUNKS, a ring */
static size_t s_pre_next, s_pre_fill;
#if MUSE_LOW_MEM
static muse_adpcm_t s_pre_enc;
#endif
static int16_t s_chunk[MUSE_AUDIO_CHUNK];
static int s_settle;

/*
 * The note being recorded (HOLD_NOTES), kept whole in case it can't go now.
 * Streaming, Hatch has had s_sent frames of it.
 */
static int16_t *s_rec;   /* MAX_FRAMES */
static size_t s_rec_n, s_sent;
static bool s_live = true;
static bool s_tried;     /* it went to Hatch */

typedef struct {
    int16_t *pcm;
    size_t frames;
    int64_t at_us;   /* when it was recorded */
    int tries;
} held_note_t;

static int s_held_count;
static volatile bool s_waiting;        /* muse_voice_notes_waiting() */
#if HOLD_NOTES
static held_note_t s_held[HELD_MAX];   /* oldest first */
static int64_t s_next_send_us;
static int64_t s_send_backoff_us = RETRY_MIN_US;
#endif

/* The i-th oldest pre-roll chunk, into out. */
static void pre_get(size_t i, int16_t *out)
{
    const pre_chunk_t *c = &s_pre[(s_pre_next + PRE_CHUNKS - s_pre_fill + i) % PRE_CHUNKS];
#if MUSE_LOW_MEM
    muse_adpcm_t dec = c->start;
    muse_adpcm_decode_block(&dec, c->adpcm, MUSE_AUDIO_CHUNK, out);
#else
    memcpy(out, c->pcm, sizeof(c->pcm));
#endif
}

/* Call after Muse plays anything: its own sound must not become pre-roll. */
static void pre_reset(void)
{
    s_pre_fill = 0;
    s_settle = SETTLE_CHUNKS;
}

/* Software gain on top of the 36 dB hardware PGA, which is already
 * maxed out: every captured chunk is amplified 4x (+12 dB) at capture
 * time, so the speech gate, the stats, the note buffer, the pre-roll
 * ring and the live stream all hear the louder signal. Saturates
 * instead of wrapping, so over-gain shows up as clipped samples in
 * the record log. Applied at capture only, never at pre-roll replay,
 * so no chunk is ever gained twice. */
static void sw_gain(int16_t *pcm, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        int x = (int)pcm[i] * 4;
        pcm[i] = x > 32767 ? 32767 : x < -32768 ? -32768 : (int16_t)x;
    }
}

/* One 20 ms idle read: feeds the pre-roll ring and the settings mic meter. */
static void idle_capture(void)
{
    if (muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }
    sw_gain(s_chunk, MUSE_AUDIO_CHUNK);
    if (s_monitor) {
        float db = muse_audio_dbfs(s_chunk, MUSE_AUDIO_CHUNK);
        s_monitor_db = db > s_monitor_db ? db : s_monitor_db * 0.9f + db * 0.1f;
    }
    if (s_settle > 0) {
        s_settle--;
        return;
    }
    pre_chunk_t *c = &s_pre[s_pre_next];
#if MUSE_LOW_MEM
    c->start = s_pre_enc;
    muse_adpcm_encode_block(&s_pre_enc, s_chunk, MUSE_AUDIO_CHUNK, c->adpcm);
#else
    memcpy(c->pcm, s_chunk, sizeof(c->pcm));
#endif
    s_pre_next = (s_pre_next + 1) % PRE_CHUNKS;
    s_pre_fill = s_pre_fill < PRE_CHUNKS ? s_pre_fill + 1 : PRE_CHUNKS;
    /* Wake-word engine, only while the voice task armed it (idle, plugged
     * in, within reach). The settle skip above already kept Muse's own
     * tail out of this chunk. */
    if (s_wake_armed && muse_wake_feed(s_chunk, MUSE_AUDIO_CHUNK)) {
        s_wake_fired = true;
    }
}

/* Non-blocking: returns true if an event of `type` arrived (others dropped). */
static bool got_event(muse_ptt_t type)
{
    muse_input_event_t ev;
    bool hit = false;
    while (xQueueReceive(s_queue, &ev, 0) == pdTRUE) {
        muse_state_poke();
        hit |= ev.type == type;
    }
    return hit;
}

/* Level stats of a recording, gathered chunk by chunk for the log. */
typedef struct {
    double acc;
    size_t frames;
    int peak, clipped;
    float floor_db;
    float tail_db[25];   /* the last 0.5 s, to see where speech ends and the release click lands */
    size_t chunks;
} rec_stats_t;

/* Adds one captured chunk (up to MUSE_AUDIO_CHUNK frames) to the stats. */
static void keep(rec_stats_t *st, const int16_t *pcm, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        int v = abs(pcm[i]);
        st->peak = v > st->peak ? v : st->peak;
        st->clipped += v >= 32000;
        st->acc += (double)pcm[i] * pcm[i];
    }
    st->frames += n;
    if (n == MUSE_AUDIO_CHUNK) {
        float db = muse_audio_dbfs(pcm, n);
        st->floor_db = db < st->floor_db ? db : st->floor_db;
        st->tail_db[st->chunks++ % 25] = db;
    }
}

/* Gives Hatch what it has room for of the note being recorded. */
static void feed_live(void)
{
#if HOLD_NOTES
    if (s_live && s_sent < s_rec_n) {
        s_sent += muse_hatch_turn_audio_wait(s_rec + s_sent, s_rec_n - s_sent, 0);
    }
#endif
}

/* Streams the note being recorded from its start, then the rest as it comes. */
static void go_live(void)
{
    muse_hatch_turn_begin();
    s_live = s_tried = true;
    s_sent = 0;
    feed_live();
}

/* One chunk of speech: into the stats, and on to Hatch, by way of the kept note if there is one. */
static void take(rec_stats_t *st, const int16_t *pcm)
{
    keep(st, pcm, MUSE_AUDIO_CHUNK);
    if (!s_rec) {
        if (s_live) {
            muse_hatch_turn_audio(pcm, MUSE_AUDIO_CHUNK);
        }
        return;
    }
    memcpy(s_rec + s_rec_n, pcm, MUSE_AUDIO_CHUNK * sizeof(int16_t));
    s_rec_n += MUSE_AUDIO_CHUNK;
    feed_live();
}

static void drop_rec(void);

/*
 * Records speech until release or MAX_SECS, starting with the pre-roll; *held
 * gets the part after the press. It streams to Hatch as it goes if it can.
 * If not, the note is kept in s_rec to send later, and streams from partway
 * if Hatch comes within reach. Returns false if Hatch failed the turn with no
 * kept note to fall back on (why says what failed). There is no start chirp:
 * anything played now would land on top of the first words.
 */
static bool record(bool barge_in, size_t *held, char *why, size_t cap)
{
    muse_state_set_mode(MUSE_MODE_LISTENING);
    muse_state_set_progress(0);
    s_rec_n = s_sent = 0;
    s_cancel_requested = false;   /* a stale tap must not kill this note */
    s_live = s_tried = false;
    if (!s_rec || (muse_hatch_ready() && !s_held_count)) {
        go_live();
    }
    muse_state_set_caption(s_live ? "LISTENING..." : "RECORDING...");
    bool heard = false, ok = true;
    bool gave_up = false;   /* Hatch failed this note: it's kept, and goes later */
    size_t speech_chunks = 0;
    char text[96];
    rec_stats_t st = { 0 };

    size_t n = 0;
    if (barge_in) {
        /* Pressed during playback: the queued tail of the reply is still sounding. */
        for (int i = 0; i < SETTLE_CHUNKS; i++) {
            muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK);
        }
    } else {
        for (size_t i = 0; i < s_pre_fill; i++) {
            pre_get(i, s_chunk);
            take(&st, s_chunk);
            if (muse_audio_dbfs(s_chunk, MUSE_AUDIO_CHUNK) > CALL_SPEECH_DB) {
                speech_chunks++;
            }
            n += MUSE_AUDIO_CHUNK;
        }
    }
    size_t pre = n;
    bool released = false;
    bool cancelled = false;
    size_t stop_at = MAX_FRAMES;
    while (n + MUSE_AUDIO_CHUNK <= stop_at) {
        if (muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK) != ESP_OK) {
            break;
        }
        sw_gain(s_chunk, MUSE_AUDIO_CHUNK);
        muse_state_set_level(muse_audio_level(s_chunk, MUSE_AUDIO_CHUNK));
        take(&st, s_chunk);
        if (muse_audio_dbfs(s_chunk, MUSE_AUDIO_CHUNK) > CALL_SPEECH_DB) {
            speech_chunks++;
        }
        /* Live transcript as the caption. A failure stops the streaming; the
         * kept note goes later, or without one the failure is the caption. */
        muse_hatch_ev_t ev;
        while (s_live && (ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            if (ev == MUSE_HATCH_EV_HEARD && text[0]) {
                heard = true;
                muse_state_set_caption("%s", text);
            } else if (ev == MUSE_HATCH_EV_ERROR) {
                ESP_LOGW(TAG, "muse: %s", text);
                muse_hatch_turn_cancel();
                s_live = false;
                gave_up = true;
                if (!s_rec) {
                    ok = false;
                    strlcpy(why, text, cap);
                    muse_state_set_caption("%s", text);
                }
            }
        }
        n += MUSE_AUDIO_CHUNK;
        bool tick = n % (MUSE_AUDIO_CHUNK * 5) == 0;
        if (tick && s_rec && !s_live && !gave_up && !s_held_count && muse_hatch_ready()) {
            ESP_LOGI(TAG, "Muse in reach: streaming the note so far");
            go_live();
        }
        muse_state_set_progress((float)n / MAX_FRAMES);
        if (!heard && ok && tick) {
            muse_state_set_caption("%s %.1fs", s_live ? "LISTENING" : "RECORDING", (double)n / MUSE_AUDIO_RATE);
        }
        /*
         * Capture runs 60-80 ms behind real time and people let go on their
         * last syllable, so keep going briefly after release.
         */
        if (!released && got_event(MUSE_PTT_UP)) {
            released = true;
            stop_at = n + TAIL_FRAMES < MAX_FRAMES ? n + TAIL_FRAMES : MAX_FRAMES;
        }
        if (s_cancel_requested) {
            s_cancel_requested = false;
            cancelled = true;
            break;
        }
    }
    if (cancelled) {
        muse_hatch_turn_cancel();
        drop_rec();
        *held = 0;
        if (cap > 0) {
            why[0] = '\0';
        }
        muse_state_set_level(0);
        return false;
    }
    if (!heard && speech_chunks < SPEECH_MIN_CHUNKS) {
        /* Silence (or only noise below the speech threshold) held
         * past the tap length: cancel the turn before it is
         * finalized, so nothing is sent or held for later. Live
         * audio already streamed is discarded with the turn. A
         * transcript (heard) overrides the energy count, so quiet
         * speech the cloud did hear still goes through. */
        ESP_LOGI(TAG, "no speech in note (%.2fs above threshold), dropped", (double)speech_chunks / 50.0);
        muse_hatch_turn_cancel();
        drop_rec();
        *held = 0;
        if (cap > 0) {
            strlcpy(why, "NO SPEECH", cap);
        }
        muse_state_set_level(0);
        return false;
    }
    muse_state_set_level(0);
    *held = n - pre;

    char tail[160];
    int tl = 0;
    size_t shown = st.chunks < 25 ? st.chunks : 25;
    for (size_t i = st.chunks - shown; i < st.chunks; i++) {
        tl += snprintf(tail + tl, sizeof(tail) - tl, " %.0f", st.tail_db[i % 25]);
    }
    ESP_LOGI(TAG, "end levels:%s", tail);
    float rms_db = st.frames ? 10.0f * log10f((float)(st.acc / st.frames / (32768.0 * 32768.0)) + 1e-10f) : -100.0f;
    ESP_LOGI(TAG, "recorded %.2fs%s: rms %.1f dBFS, peak %.1f dBFS, floor %.1f dBFS, %d clipped, gain %d dB, speech %.2fs",
             (double)n / MUSE_AUDIO_RATE, released ? "" : " (max)", rms_db < -100.0f ? -100.0f : rms_db,
             20.0 * log10((st.peak + 1) / 32768.0), st.floor_db, st.clipped, muse_settings_mic_gain(),
             (double)speech_chunks / 50.0);
    return ok;
}

static void go_idle(const char *caption);

/*
 * Plays Hatch's reply as it arrives, with its text as the caption. Returns
 * true if interrupted by a new press. *delivered: the VM has the note.
 * `waiting` is the caption until there's a transcript or reply.
 */
static bool hatch_reply_as(bool *delivered, const char *waiting)
{
    muse_state_set_mode(MUSE_MODE_THINKING);
    muse_state_set_caption("%s", waiting);
    static int16_t buf[MUSE_AUDIO_CHUNK];
    static const int16_t silence[MUSE_AUDIO_CHUNK];
    char text[96];
    static char page[MUSE_CAPTION_MAX];
    bool done = false, speaking = false, replied = false;
    size_t played = 0;
    int64_t t0 = esp_timer_get_time();
    *delivered = false;
    s_reply_failed = false;
    for (;;) {
        muse_hatch_ev_t ev;
        while ((ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            switch (ev) {
            case MUSE_HATCH_EV_HEARD:
                if (!speaking && !replied) {
                    muse_state_set_caption("\"%s\"", text);
                }
                break;
            case MUSE_HATCH_EV_SENT:
                *delivered = true;
                if (!speaking && !replied) {
                    muse_state_set_caption("NOTE SENT - WAITING FOR MUSE");
                }
                break;
            case MUSE_HATCH_EV_REPLY:
                replied = *delivered = true;
                /* Once speech starts, the caption follows it. The event only
                 * has room for the page's start; the page itself comes below. */
                if (!speaking && !muse_hatch_turn_caption(played, page, sizeof(page))) {
                    muse_state_set_caption("%s", text);
                }
                break;
            case MUSE_HATCH_EV_DONE:
                done = *delivered = true;
                break;
            case MUSE_HATCH_EV_ERROR:
                ESP_LOGW(TAG, "muse: %s", text);
                s_reply_failed = true;
                strlcpy(s_last_error, text, sizeof(s_last_error));
                muse_state_set_level(0);
                go_idle(text);
                return false;
            default:
                break;
            }
        }
        if (s_hangup_requested) {
            ESP_LOGI(TAG, "reply hung up");
            muse_hatch_turn_cancel();
            muse_state_set_level(0);
            return true;
        }
        if (got_event(MUSE_PTT_DOWN)) {
            ESP_LOGI(TAG, "reply interrupted");
            muse_hatch_turn_cancel();
            muse_state_set_level(0);
            return true;
        }
        size_t n = muse_hatch_turn_read(buf, MUSE_AUDIO_CHUNK, speaking || done ? 0 : 20);
        if (n) {
            if (!speaking) {
                speaking = true;
                muse_state_set_mode(MUSE_MODE_SPEAKING);
                ESP_LOGI(TAG, "reply audio after %.2fs", (esp_timer_get_time() - t0) / 1e6);
            }
            muse_state_set_level(muse_audio_level(buf, n));
            muse_audio_write(buf, n);
            played += n;
        } else if (done) {
            break;
        } else if (speaking) {
            /* Between messages: keep the speaker fed so it doesn't replay stale DMA. */
            muse_state_set_level(0);
            muse_audio_write(silence, MUSE_AUDIO_CHUNK);
        }
        /* No voice barge-in: only a BOOT hold (PTT_DOWN, checked above)
         * interrupts a reply. The mic is not read during playback, so
         * there is no voice-vs-speaker judgment to run. */
        /* The page being said, or before the speech the reply's opening page. */
        if ((speaking || replied) && muse_hatch_turn_caption(played, page, sizeof(page))) {
            muse_state_set_caption("%s", page);
        }
    }
    muse_state_set_level(0);
    ESP_LOGI(TAG, "muse reply: %.2fs of audio, %.2fs total", (double)played / MUSE_AUDIO_RATE,
             (esp_timer_get_time() - t0) / 1e6);
    /* page is static and now holds the final page; the closing
     * check in call_run reads it from here. */
    strlcpy(s_last_reply, page, sizeof(s_last_reply));
    if (!played) {
        /* No speech (TTS unavailable): leave the reply text up for a moment. */
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    return false;
}

static bool hatch_reply(bool *delivered)
{
    return hatch_reply_as(delivered, "SENDING VOICE NOTE");
}

/* A push from Muse: wake the screen and play it like a reply. True if a
 * press interrupted it, to be recorded as one. */
static bool play_push(void)
{
    ESP_LOGI(TAG, "playing a push from Muse");
    if (!muse_settings_push_announce()) {
        /* Quiet push: the message is still received (text and dialog),
         * but never spoken, so automatic tasks don't wake anyone.
         * The speaker is muted only for this turn; the saved volume
         * is untouched, the screen stays dark, and calls are
         * unaffected (they never go through this path). */
        int vol = muse_settings_volume();
        muse_audio_set_volume(0);
        muse_wifi_power(MUSE_WIFI_FULL);
        bool delivered;
        bool r = hatch_reply_as(&delivered, "");
        muse_audio_set_volume(vol);
        return r;
    }
    muse_state_set_asleep(false);
    muse_state_poke();
    muse_wifi_power(MUSE_WIFI_FULL);
    bool delivered;
    return hatch_reply_as(&delivered, "");
}

static void go_idle(const char *caption)
{
    muse_state_set_progress(0);
    muse_state_set_mode(MUSE_MODE_IDLE);
    muse_state_set_caption("%s", caption);
}

/* Why a press can't go to Hatch; voice notes only go there. */
static const char *not_ready_reason(void)
{
    muse_hatch_status_t st;
    muse_hatch_status(&st);
    switch (st.state) {
    case MUSE_HATCH_NOT_SET: return "SET UP MUSE FIRST";
    case MUSE_HATCH_OFFLINE: return "NO WI-FI";
    default: return "CAN'T REACH MUSE";
    }
}

static volatile bool s_resting;

/*
 * Asleep on battery with nothing to do: power the codecs down (the loop rests
 * Wi-Fi alongside). They come back within one wait on waking, and a talk press
 * that woke Muse is kept for the loop (a note may follow). On USB power
 * nothing changes asleep.
 */
static void set_resting(bool rest)
{
    if (rest == s_resting) {
        return;
    }
    s_resting = rest;
    muse_audio_power(!rest);
    muse_hatch_set_resting(rest);
    if (rest) {
        /* Presses from before sleeping are stale, but not one that just woke Muse. */
        muse_input_event_t ev;
        while (xQueueReceive(s_queue, &ev, 0) == pdTRUE) {
            if (ev.wake) {
                xQueueSendToFront(s_queue, &ev, 0);
                break;
            }
        }
    } else {
        pre_reset();
    }
    ESP_LOGI(TAG, "%s", rest ? "resting: codecs off, Wi-Fi modem sleep" : "awake: codecs on");
}

/*
 * A press that woke the screen: listen from now, into the pre-roll, and
 * record only if it's still held after a tap's length. False if it was let
 * go sooner: it only woke Muse.
 */
static bool held_on_waking(void)
{
    s_settle = 0;   /* nothing played: no tail of Muse's own to skip */
    for (size_t n = 0; n < MIN_HELD_FRAMES; n += MUSE_AUDIO_CHUNK) {
        if (got_event(MUSE_PTT_UP)) {
            return false;
        }
        idle_capture();
    }
    return !got_event(MUSE_PTT_UP);
}

/* Discards the note just captured. The buffer itself is never
 * freed: once taken (at startup, or by the first note that
 * manages it), it stays for the life of the device, because a
 * fragmented heap cannot give a fresh 960 KB block back. */
static void drop_rec(void)
{
    s_rec_n = 0;
}

#if HOLD_NOTES
/* True if a press is queued, left there for the loop; releases ahead of it are dropped. */
static bool press_waiting(void)
{
    muse_input_event_t ev;
    while (xQueuePeek(s_queue, &ev, 0) == pdTRUE) {
        if (ev.type == MUSE_PTT_DOWN) {
            return true;
        }
        xQueueReceive(s_queue, &ev, 0);
    }
    return false;
}

static void back_off(void)
{
    s_next_send_us = esp_timer_get_time() + s_send_backoff_us;
    s_send_backoff_us = s_send_backoff_us * 2 > RETRY_MAX_US ? RETRY_MAX_US : s_send_backoff_us * 2;
}

static void drop_oldest(void);

/* Saves the note just recorded to send later. `tried`: Hatch failed it, so it waits out the backoff.
 * Only the latest note is kept: once the copy is safely allocated, any
 * previously held note is dropped to make room for this one. */
static void hold_rec(bool tried)
{
    if (!s_rec_n) {   /* nothing captured: realloc to 0 would free s_rec and return NULL */
        drop_rec();
        go_idle("");
        return;
    }
    int16_t *pcm = heap_caps_malloc(s_rec_n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm) {
        /* The note is lost, but the device keeps working: the
         * reusable note buffer stays, and the next note can try. */
        ESP_LOGW(TAG, "no memory to save a note for later");
        drop_rec();
        go_idle("COULDN'T SAVE THE NOTE");
        return;
    }
    memcpy(pcm, s_rec, s_rec_n * sizeof(int16_t));
    if (s_held_count >= HELD_MAX) {
        drop_oldest();   /* keep only the latest: the previous note goes */
    }
    s_held[s_held_count++] = (held_note_t){
        .pcm = pcm,
        .frames = s_rec_n,
        .at_us = esp_timer_get_time(),
        .tries = tried,
    };
    if (tried) {
        back_off();
    }
    ESP_LOGI(TAG, "saved a %.1fs note to send later (%d waiting)", (double)s_rec_n / MUSE_AUDIO_RATE, s_held_count);
    go_idle(tried ? "SAVED, WILL TRY AGAIN" : "SAVED, SENDS WHEN ONLINE");
}

static void drop_oldest(void)
{
    free(s_held[0].pcm);
    s_held_count--;
#if HELD_MAX > 1
    /* With a cap of 1 there is never a second element to shift, and
     * keeping this line would read s_held + 1 past a 1-element array,
     * which GCC rejects under -Werror=stringop-overread. */
    memmove(s_held, s_held + 1, s_held_count * sizeof(s_held[0]));
#endif
    if (!s_held_count) {
        s_next_send_us = 0;
        s_send_backoff_us = RETRY_MIN_US;
    }
}

typedef enum {
    FED,
    FEED_FAILED,    /* the turn failed, or Hatch stopped taking audio */
    FEED_PRESSED,   /* a press is queued: it goes first */
} feed_t;

/*
 * Gives Hatch a note from *sent on, waiting for room as it goes, then ends
 * the turn. Stops for a press only if `yield`.
 */
static feed_t feed_rest(const int16_t *pcm, size_t frames, size_t *sent, bool yield)
{
    char text[96];
    int64_t moved = esp_timer_get_time();
    for (;;) {
        muse_hatch_ev_t ev;
        while ((ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            if (ev == MUSE_HATCH_EV_ERROR) {
                ESP_LOGW(TAG, "muse: %s", text);
                return FEED_FAILED;
            }
        }
        if (*sent == frames) {
            break;
        }
        if (yield && press_waiting()) {
            return FEED_PRESSED;
        }
        size_t n = muse_hatch_turn_audio_wait(pcm + *sent, frames - *sent, 100);
        *sent += n;
        int64_t now = esp_timer_get_time();
        if (n) {
            moved = now;
        } else if (now - moved > STALL_US) {
            ESP_LOGW(TAG, "Muse stopped taking the note");
            return FEED_FAILED;
        }
    }
    muse_hatch_turn_end();
    return FED;
}

/* Asleep: waits for the VM to have the note just sent. */
static bool wait_delivered(void)
{
    char text[96];
    int64_t give_up = esp_timer_get_time() + ACK_WAIT_US;
    while (esp_timer_get_time() < give_up) {
        muse_hatch_ev_t ev;
        while ((ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            if (ev == MUSE_HATCH_EV_SENT || ev == MUSE_HATCH_EV_REPLY || ev == MUSE_HATCH_EV_DONE) {
                return true;
            }
            if (ev == MUSE_HATCH_EV_ERROR) {
                ESP_LOGW(TAG, "muse: %s", text);
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGW(TAG, "no word that the note arrived");
    return false;
}

/*
 * Sends the oldest saved note. Awake, it's a turn like any other, reply and
 * all; asleep (quiet), it's done once the VM has the note, and the reply
 * waits in the app. A press while it goes up comes first, and the note stays
 * saved. Returns true if a press interrupted the reply.
 */
static bool send_held(bool quiet)
{
    held_note_t *h = &s_held[0];
    ESP_LOGI(TAG, "sending a saved note%s: %.1fs, from %llds ago, try %d", quiet ? " (asleep)" : "",
             (double)h->frames / MUSE_AUDIO_RATE, (long long)((esp_timer_get_time() - h->at_us) / 1000000),
             h->tries + 1);
    if (!quiet) {
        muse_state_set_mode(MUSE_MODE_THINKING);
        muse_state_set_caption("SENDING SAVED NOTE");
    }
    muse_hatch_turn_begin();
    size_t sent = 0;
    feed_t fed = feed_rest(h->pcm, h->frames, &sent, true);
    bool delivered = false, interrupted = false;
    if (fed == FED && quiet) {
        delivered = wait_delivered();
    } else if (fed == FED) {
        interrupted = hatch_reply(&delivered);
    }
    if (fed != FED || quiet) {
        muse_hatch_turn_cancel();   /* asleep, the reply is left for the app */
    }
    if (fed == FEED_PRESSED) {
        if (!quiet) {
            go_idle("");
        }
        return false;
    }
    const char *caption;
    if (delivered || interrupted) {
        drop_oldest();
        s_next_send_us = 0;   /* the next one right away */
        s_send_backoff_us = RETRY_MIN_US;
        if (quiet) {
            muse_state_set_caption("SAVED NOTE SENT");
        }
        return interrupted;
    } else if (++h->tries >= HELD_TRIES) {
        ESP_LOGW(TAG, "giving up on a saved note after %d tries", h->tries);
        drop_oldest();
        caption = "COULDN'T SEND A SAVED NOTE";
    } else {
        back_off();
        caption = "SAVED NOTE: WILL TRY AGAIN";
    }
    if (quiet) {
        muse_state_set_caption("%s", caption);
    } else {
        go_idle(caption);
    }
    return false;
}

/*
 * Whether to send a saved note now: Hatch within reach and the backoff over.
 * Coming within reach starts the backoff over. Keeps s_waiting too.
 */
static bool held_due(void)
{
    static bool was_ready = true;
    if (!s_held_count) {
        was_ready = true;
        s_waiting = false;
        return false;
    }
    int64_t now = esp_timer_get_time();
    s_waiting = now - s_held[0].at_us < HELD_KEEP_US;
    bool ready = muse_hatch_ready();
    if (ready && !was_ready) {
        s_next_send_us = 0;
        s_send_backoff_us = RETRY_MIN_US;
    }
    was_ready = ready;
    return ready && now >= s_next_send_us;
}
#endif

/*
 * After the release: the rest of the note goes to Hatch and the reply plays,
 * or if it can't go now it's saved to send later. Returns true if a press
 * interrupted the reply. Every utterance that actually reaches the
 * cloud gets the two-beep sent chirp before the reply plays; a note
 * held for later gets none. Waking chirps up, hanging up chirps down,
 * and the caller puts the face back to listening instead of idle.
 */
static bool finish_note(bool in_call, bool *replied_out)
{
    bool delivered;
    s_reply_failed = false;
    s_last_reply[0] = '\0';
#if HOLD_NOTES
    if (s_rec) {
        bool fed = false, interrupted = false;
        delivered = false;
        if (s_live && s_sent == s_rec_n) {
            muse_hatch_turn_end();   /* finalized first; the sent chirp plays only if fed */
            fed = true;
        }
        if (s_live && !fed) {
            /* Hatch is behind (still connecting, say): the rest from the kept note. */
            muse_state_set_mode(MUSE_MODE_THINKING);
            muse_state_set_caption("SENDING VOICE NOTE");
            fed = feed_rest(s_rec, s_rec_n, &s_sent, false) == FED;
            if (!fed) {
                muse_hatch_turn_cancel();
            }
        }
        if (fed) {
            muse_audio_chirp_sent();   /* reached the cloud; held notes get none */
            interrupted = hatch_reply(&delivered);
        }
        if (delivered || interrupted) {
            drop_rec();
        } else {
            hold_rec(s_tried);
        }
        if (replied_out) {
            *replied_out = delivered && !s_reply_failed;
        }
        return interrupted;
    }
#endif
    muse_hatch_turn_end();   /* before the sent chirp, which takes ~230 ms */
    muse_audio_chirp_sent();
    {
        bool interrupted = hatch_reply(&delivered);
        if (replied_out) {
            *replied_out = delivered && !s_reply_failed;
        }
        return interrupted;
    }
}

static bool can_record(void);

/* Drains the input queue once, reporting whether a BOOT press or release
 * arrived (others dropped). Like got_event, any event pokes the state,
 * which keeps the screen awake through an active call. */
static void drain_ptt(bool *down, bool *up)
{
    muse_input_event_t ev;
    *down = *up = false;
    while (xQueueReceive(s_queue, &ev, 0) == pdTRUE) {
        muse_state_poke();
        if (ev.type == MUSE_PTT_DOWN) {
            *down = true;
        } else if (ev.type == MUSE_PTT_UP) {
            *up = true;
        }
    }
}

/*
 * One call utterance: the same kept-note and live-stream machinery as
 * record(), but the end is voice activity, not a release. No pre-roll
 * seeding: the ring still holds the wake word itself, which must not go
 * as content. `barge_in` skips the speaker tail after an interrupted
 * reply, or the wake chirp on the first capture, as record() does. A BOOT hold during capture disables the
 * silence-gap end (release ends it instead); a BOOT release with speech
 * ends it at once. The silence watchdog (`*silence_deadline`) is reset
 * on speech and checked while waiting for it; the caller owns it.
 */
static call_outcome_t capture_call_utterance(bool barge_in, int64_t *silence_deadline)
{
    muse_state_set_mode(MUSE_MODE_LISTENING);
    muse_state_set_progress(0);
    s_rec_n = s_sent = 0;
    s_cancel_requested = false;
    s_last_heard[0] = '\0';
    s_live = s_tried = false;
    if (!s_rec) {
        s_rec = heap_caps_malloc(MAX_FRAMES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rec) {
            ESP_LOGE(TAG, "no memory to keep a note (psram free %u, largest %u)",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
            muse_state_set_caption("NO MEMORY FOR NOTE");
            return CALL_HANGUP;
        }
    }
    if (muse_hatch_ready() && !s_held_count) {
        go_live();
    }
    muse_state_set_caption(s_live ? "LISTENING..." : "RECORDING...");
    bool heard = false, ok = true;
    bool gave_up = false;
    char text[96];
    rec_stats_t st = { 0 };

    size_t n = 0;
    if (barge_in) {
        for (int i = 0; i < SETTLE_CHUNKS; i++) {
            muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK);
        }
    }
    bool speech = false, manual = muse_input_talk_down();
    size_t speech_chunks = 0;
    size_t gap = 0;
    bool released = false;
    size_t stop_at = MAX_FRAMES;
    while (n + MUSE_AUDIO_CHUNK <= stop_at) {
        if (muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK) != ESP_OK) {
            /* A dead mic must not spin: the silence watchdog hangs up. */
            vTaskDelay(pdMS_TO_TICKS(20));
            if (esp_timer_get_time() > *silence_deadline) {
                muse_hatch_turn_cancel();
                drop_rec();
                muse_state_set_level(0);
                return CALL_HANGUP;
            }
            continue;
        }
        sw_gain(s_chunk, MUSE_AUDIO_CHUNK);
        float db = muse_audio_dbfs(s_chunk, MUSE_AUDIO_CHUNK);
        if (db > CALL_SPEECH_DB) {
            speech_chunks++;
        }
        muse_state_set_level(muse_audio_level(s_chunk, MUSE_AUDIO_CHUNK));
        take(&st, s_chunk);
        muse_hatch_ev_t ev;
        while (s_live && (ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            if (ev == MUSE_HATCH_EV_HEARD && text[0]) {
                heard = true;
                strlcpy(s_last_heard, text, sizeof(s_last_heard));
                muse_state_set_caption("%s", text);
            } else if (ev == MUSE_HATCH_EV_ERROR) {
                ESP_LOGW(TAG, "muse: %s", text);
                muse_hatch_turn_cancel();
                s_live = false;
                gave_up = true;
                if (!s_rec) {
                    ok = false;
                    muse_state_set_caption("%s", text);
                }
            }
        }
        n += MUSE_AUDIO_CHUNK;
        bool tick = n % (MUSE_AUDIO_CHUNK * 5) == 0;
        if (tick && s_rec && !s_live && !gave_up && !s_held_count && muse_hatch_ready()) {
            ESP_LOGI(TAG, "Muse in reach: streaming the note so far");
            go_live();
        }
        muse_state_set_progress((float)n / MAX_FRAMES);
        if (!heard && ok && tick) {
            muse_state_set_caption("%s %.1fs", s_live ? "LISTENING" : "RECORDING", (double)n / MUSE_AUDIO_RATE);
        }
        bool down, up;
        drain_ptt(&down, &up);
        if (down) {
            manual = true;
        }
        if (s_hangup_requested) {
            muse_hatch_turn_cancel();
            drop_rec();
            muse_state_set_level(0);
            return CALL_HANGUP;
        }
        if (s_cancel_requested) {
            s_cancel_requested = false;
            muse_hatch_turn_cancel();
            drop_rec();
            muse_state_set_level(0);
            return CALL_END;
        }
        if (!speech) {
            if (db > CALL_SPEECH_DB) {
                speech = true;
                gap = 0;
                muse_state_poke();
                *silence_deadline = esp_timer_get_time() + CALL_SILENCE_US;
            } else if (esp_timer_get_time() > *silence_deadline) {
                muse_hatch_turn_cancel();
                drop_rec();
                muse_state_set_level(0);
                return CALL_HANGUP;
            } else if (up) {
                /* A BOOT hold released without speech: the user is
                 * there, so the watchdog starts over. */
                manual = false;
                *silence_deadline = esp_timer_get_time() + CALL_SILENCE_US;
            }
            continue;
        }
        /* Speech heard: any loud chunk pushes the watchdog out. */
        if (db > CALL_SILENCE_DB) {
            gap = 0;
            *silence_deadline = esp_timer_get_time() + CALL_SILENCE_US;
        } else if (!manual) {
            gap++;
        }
        if (up && !released) {
            /* BOOT let go: keep going briefly, as after a release. */
            released = true;
            stop_at = n + TAIL_FRAMES < MAX_FRAMES ? n + TAIL_FRAMES : MAX_FRAMES;
        }
        if (!manual && gap >= CALL_GAP_CHUNKS) {
            break;
        }
        if (n + MUSE_AUDIO_CHUNK > stop_at) {
            break;
        }
    }
    muse_state_set_level(0);
    if (!ok) {
        drop_rec();
        return CALL_CANCELLED;
    }
    if (n < MIN_HELD_FRAMES || (!heard && speech_chunks < SPEECH_MIN_CHUNKS)) {
        /* A cough, a click, or silence, not an utterance: drop it,
         * stay in the call. A transcript (heard) overrides the
         * energy count, so quiet speech the cloud did hear still
         * goes through. */
        ESP_LOGI(TAG, "utterance dropped: no speech (%.2fs above threshold)", (double)speech_chunks / 50.0);
        muse_hatch_turn_cancel();
        drop_rec();
        return CALL_CANCELLED;
    }
    char tail[160];
    int tl = 0;
    size_t shown = st.chunks < 25 ? st.chunks : 25;
    for (size_t i = st.chunks - shown; i < st.chunks; i++) {
        tl += snprintf(tail + tl, sizeof(tail) - tl, " %.0f", st.tail_db[i % 25]);
    }
    ESP_LOGI(TAG, "end levels:%s", tail);
    float rms_db = st.frames ? 10.0f * log10f((float)(st.acc / st.frames / (32768.0 * 32768.0)) + 1e-10f) : -100.0f;
    ESP_LOGI(TAG, "call utterance %.2fs: rms %.1f dBFS, peak %.1f dBFS, floor %.1f dBFS, %d clipped",
             (double)n / MUSE_AUDIO_RATE, rms_db < -100.0f ? -100.0f : rms_db,
             20.0 * log10((st.peak + 1) / 32768.0), st.floor_db, st.clipped);
    return CALL_UTTERANCE;
}

/*
 * A wake-word call, entered from the voice task's idle branch. One
 * utterance first: follow-up listening is enabled only after Muse
 * actually replies. An utterance with no reply (offline, held,
 * can't reach, error, or dropped before any reply) ends the call
 * instead of looping back to listening, so recording never switches
 * on/off repeatedly on a dead connection. A reply interrupted by
 * BOOT still starts the next utterance at once.
 */
static void call_run(void)
{
    s_call = true;
    s_hangup_requested = false;
    s_wake_armed = false;
    if (!can_record()) {
        /* can_record already said why; no chirp, no call. */
        s_call = false;
        return;
    }
    muse_state_set_asleep(false);
    muse_state_poke();
    muse_wifi_power(MUSE_WIFI_FULL);
    muse_audio_chirp(1);
    pre_reset();
    ESP_LOGI(TAG, "call started");
    int64_t silence_deadline = esp_timer_get_time() + CALL_SILENCE_US;
    /* First capture follows the rising chirp: skip its 200 ms tail
     * the same way a capture after an interrupted reply does, so
     * the chirp cannot false-trigger speech and cut a silent first
     * utterance short after the 1.5 s gap. Reset to false after
     * the first capture, so follow-up captures are unaffected. */
    bool barge_in = true;
    bool got_reply = false;
    const char *end_caption = "";
    for (;;) {
        call_outcome_t u = capture_call_utterance(barge_in, &silence_deadline);
        barge_in = false;
        if (u == CALL_HANGUP) {
            break;
        }
        if (u == CALL_END) {
            /* Cancel tap: end the whole call, in any phase. Coughs
             * and too-short utterances still return CALL_CANCELLED
             * and stay in follow-up listening. */
            break;
        }
        if (u == CALL_CANCELLED) {
            /* Dropped utterance (cough, too short): before any reply
             * this ends the wake call instead of recapturing. Once a
             * reply has enabled follow-up, it only drops that
             * utterance and listening continues. */
            if (!got_reply) {
                break;
            }
            silence_deadline = esp_timer_get_time() + CALL_SILENCE_US;
            pre_reset();
            continue;
        }
        bool replied = false;
        bool interrupted = finish_note(true, &replied);
        if (s_hangup_requested) {
            break;
        }
        if (interrupted) {
            /* BOOT barge-in: the user is speaking, so listen
             * again at once. */
            barge_in = true;
            silence_deadline = esp_timer_get_time() + CALL_SILENCE_US;
            pre_reset();
            continue;
        }
        if (!replied) {
            /* No reply (offline, held, can't reach, error): stop after
             * this one recording instead of looping back to listening.
             * Keep the failure caption when there was one, so the end
             * of the call still says why. */
            if (s_reply_failed) {
                end_caption = s_last_error;
            }
            break;
        }
        if (reply_is_closing(s_last_reply) || reply_is_closing(s_last_heard)) {
            /* A closing exchange: the user said goodbye, or Muse
             * answered with one. End the call instead of opening
             * follow-up listening; saying Jarvis again, or BOOT,
             * starts a new turn if the user wants to continue. */
            break;
        }
        /* Reply delivered: follow-up listening is enabled from here.
         * A later utterance with no reply still ends the call. */
        got_reply = true;
        muse_state_make_happy();
        muse_state_set_mode(MUSE_MODE_LISTENING);
        muse_state_poke();
        silence_deadline = esp_timer_get_time() + CALL_SILENCE_US;
        pre_reset();
    }
    s_call = false;
    s_hangup_requested = false;
    muse_audio_chirp(0);
    go_idle(end_caption);
    pre_reset();
    ESP_LOGI(TAG, "call ended");
}

/*
 * Whether a press can record: Hatch within reach, or (HOLD_NOTES) set up
 * with room to save the note. If not, the caption says why.
 */
static bool can_record(void)
{
    if (!muse_wifi_connected()) {
        muse_wifi_apply();   /* retry now, not after the backoff */
    }
    bool ready = muse_hatch_ready();
#if HOLD_NOTES
    muse_hatch_status_t st;
    muse_hatch_status(&st);
    if (st.state == MUSE_HATCH_NOT_SET) {
        go_idle("SET UP MUSE FIRST");
        return false;
    }
    /* No cap refusal: with HELD_MAX 1, a new note that ends up held
     * drops the previous held note instead of being refused. */
    if (ready && s_held_count) {
        s_next_send_us = 0;   /* the saved ones go first, right after */
    }
    /* Reuse the buffer reserved at startup: allocating a fresh one here
     * every call would leak the reserved block and, once free PSRAM
     * drops below one note, overwrite s_rec with NULL - the NO MEMORY
     * FOR NOTE that kills every later call until reboot. */
    if (!s_rec) {
        s_rec = heap_caps_malloc(MAX_FRAMES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rec) {
            ESP_LOGE(TAG, "no memory to keep a note (psram free %u, largest %u)",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        }
    }
    if (s_rec) {
        /* Once: the 30 s note buffer plus wake/AFE against free heap. */
        static bool logged;
        if (!logged) {
            logged = true;
            ESP_LOGI(TAG, "note buffer %u KB: free internal %u, psram %u",
                     (unsigned)(MAX_FRAMES * sizeof(int16_t) / 1024),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        }
    }
#endif
    if (!ready && !s_rec) {
        go_idle(not_ready_reason());
        return false;
    }
    return true;
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/*
 * One soft snore, about 0.8 s, made up as it plays: a fluttering rumble that
 * rises through the inhale (60-90 Hz, with harmonics a small speaker can
 * reproduce, over breathy low-passed noise), then a quieter exhale of air.
 * Stops early for a press or if drowsing is interrupted, so it never holds up
 * a recording.
 */
static void play_snore(void)
{
    enum { IN_FRAMES = MUSE_AUDIO_RATE * 55 / 100, OUT_FRAMES = MUSE_AUDIO_RATE / 4 };
    static int which;
    static uint32_t seed = 0x9E3779B9u;
    float base_hz = which++ & 1 ? 70.0f : 62.0f;   /* two in a row don't sound the same */
    float phase = 0, lp_in = 0, lp_out = 0;
    int16_t buf[MUSE_AUDIO_CHUNK];
    for (int done = 0; done < IN_FRAMES + OUT_FRAMES; done += MUSE_AUDIO_CHUNK) {
        if (uxQueueMessagesWaiting(s_queue) || (muse_state_sleepy() == 0 && !muse_state_asleep())) {
            break;
        }
        for (int i = 0; i < MUSE_AUDIO_CHUNK; i++) {
            int n = done + i;
            float t = (float)n / MUSE_AUDIO_RATE;
            seed = seed * 1664525u + 1013904223u;
            float noise = (float)(int32_t)seed / 2147483648.0f;
            float v = 0;
            if (n < IN_FRAMES) {
                float x = (float)n / IN_FRAMES;
                float env = sinf((float)M_PI * x);
                env *= env;
                phase += 2.0f * (float)M_PI * (base_hz + 20.0f * x) / MUSE_AUDIO_RATE;
                float tone = (sinf(phase) + 0.6f * sinf(2 * phase) + 0.35f * sinf(3 * phase)
                              + 0.2f * sinf(4 * phase)) / 2.15f;
                float flutter = 0.55f + 0.45f * sinf(2.0f * (float)M_PI * 25.0f * t);
                lp_in += 0.18f * (noise - lp_in);   /* one pole, ~500 Hz */
                v = env * (0.75f * tone * flutter + 0.5f * lp_in);
            } else if (n < IN_FRAMES + OUT_FRAMES) {
                float y = (float)(n - IN_FRAMES) / OUT_FRAMES;
                lp_out += 0.3f * (noise - lp_out);  /* ~900 Hz */
                v = 0.35f * sinf((float)M_PI * y) * lp_out;
            }
            buf[i] = (int16_t)(v * SNORE_AMP);
        }
        muse_audio_write(buf, MUSE_AUDIO_CHUNK);
    }
}

static void voice_task(void *arg)
{
    bool pending_down = false;
    muse_audio_selftest();
    for (;;) {
        bool wake = false;
        if (!pending_down) {
            muse_input_event_t ev;
            bool asleep = muse_state_asleep();
            bool battery = muse_state_on_battery();
            bool rest = asleep && battery && !s_chirp && !s_mp3test && !s_loopback;
#if HOLD_NOTES
            /* A press goes first: send_held() leaves it queued and returns
             * without backing off, so retrying before it's read would spin. */
            if (held_due() && !press_waiting()) {
                set_resting(false);   /* full power while it goes, even asleep */
                muse_wifi_power(MUSE_WIFI_FULL);
                pending_down = send_held(asleep);
                pre_reset();
                if (!pending_down && !asleep && muse_state_mode(NULL) != MUSE_MODE_IDLE) {
                    muse_state_make_happy();
                    go_idle("");
                }
                continue;
            }
#endif
            set_resting(rest);
            /* On battery Wi-Fi rests with the codecs, and between turns dozes
             * between beacons as xiaozhi's idle does; a press wakes it fully. */
            muse_wifi_power(rest ? MUSE_WIFI_REST : battery ? MUSE_WIFI_DOZE : MUSE_WIFI_FULL);
            if (rest) {
                /* Wait for waking, USB power or a bench request without
                 * polling (the timeout is only a backstop, or a look for
                 * Wi-Fi to send saved notes), so the CPU can stay asleep. */
                muse_state_wait_awake(s_waiting ? HELD_POLL_MS : REST_BACKSTOP_MS);
                continue;
            }
            if (s_chirp) {
                s_chirp = false;
                muse_audio_chirp(1);
                pre_reset();
            }
            if (s_snore_ms) {
                uint32_t asked = s_snore_ms;
                s_snore_ms = 0;
                if (now_ms() - asked < SNORE_STALE_MS && muse_settings_speaker_on()) {
                    play_snore();
                    pre_reset();
                }
            }
            if (s_mp3test) {
                s_mp3test = false;
                int16_t *pcm = NULL;
                size_t n = muse_hatch_mp3_selftest(&pcm);
                for (size_t i = 0; i < n; i += MUSE_AUDIO_CHUNK) {
                    muse_audio_write(pcm + i, n - i < MUSE_AUDIO_CHUNK ? n - i : MUSE_AUDIO_CHUNK);
                }
                free(pcm);
                pre_reset();
            }
            if (s_loopback) {
                s_loopback = false;
                muse_audio_loopback_test(muse_settings_volume());
                pre_reset();
            }
            /* Wake-word arming: idle, plugged in, within reach. The
             * screen may be dark on USB; on battery this never arms,
             * and resting (battery asleep) never reaches this read. */
            s_wake_armed = (!muse_state_on_battery() || muse_settings_wake_on_battery()) && muse_hatch_ready()
                && muse_state_mode(NULL) == MUSE_MODE_IDLE;
            /* The 20 ms read paces this loop. */
            idle_capture();
            if (xQueueReceive(s_queue, &ev, 0) != pdTRUE) {
                if (s_wake_fired) {
                    s_wake_fired = false;
                    muse_hatch_push_drop();   /* the call replaces the push */
                    call_run();
                    pre_reset();
                    continue;
                }
                if (muse_hatch_push_take()) {
                    pending_down = play_push();
                    pre_reset();
                    if (!pending_down && muse_state_mode(NULL) != MUSE_MODE_IDLE) {
                        muse_state_make_happy();
                        go_idle("");
                    }
                }
                continue;
            }
            /* A press wins over a wake word in the same instant, and the
             * armed flag must not leak into the waking-press path. */
            s_wake_fired = false;
            s_wake_armed = false;
            muse_hatch_push_drop();   /* a press goes first; its turn replaces the push's */
            muse_state_poke();
            if (ev.type != MUSE_PTT_DOWN) {
                continue;
            }
            wake = ev.wake;
        }
        if (wake && !held_on_waking()) {
            continue;   /* a tap: it only woke Muse */
        }
        muse_wifi_power(MUSE_WIFI_FULL);
        if (!can_record()) {
            pending_down = false;
            continue;
        }
        size_t held;
        char why[96];
        bool ok = record(pending_down, &held, why, sizeof(why));
        pending_down = false;
        if (held < MIN_HELD_FRAMES && !wake) {
            muse_hatch_turn_cancel();
            drop_rec();
            pre_reset();
            go_idle("");
            continue;
        }
        if (!ok) {
            pre_reset();
            go_idle(why);   /* Hatch failed it while recording */
            continue;
        }
        pending_down = finish_note(false, NULL);
        pre_reset();
        if (!pending_down && muse_state_mode(NULL) != MUSE_MODE_IDLE) {
            muse_state_make_happy();
            go_idle("");
        }
    }
}

/* Takes the note buffer while the heap is still clean. App startup
 * calls this before the UI: by voice start, the UI's avatar cells,
 * image buffer, strips and widgets (all in PSRAM) have already
 * carved the heap, and the largest free block can be smaller than
 * one note even though total free PSRAM looks ample. Kept for the
 * life of the device; held notes get their own exact-size copy. */
void muse_voice_reserve_note_buffer(void)
{
#if HOLD_NOTES
    if (!s_rec) {
        s_rec = heap_caps_malloc(MAX_FRAMES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_rec) {
            ESP_LOGI(TAG, "note buffer %u KB reserved: free internal %u, psram %u",
                     (unsigned)(MAX_FRAMES * sizeof(int16_t) / 1024),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        } else {
            ESP_LOGW(TAG, "note buffer not reserved at start; each note will try again");
        }
    }
#endif
}

esp_err_t muse_voice_start(QueueHandle_t queue)
{
    s_queue = queue;
    s_pre = heap_caps_malloc(PRE_CHUNKS * sizeof(pre_chunk_t), MUSE_BIG_CAPS);
    if (!s_pre || muse_audio_init(muse_settings_volume(), muse_settings_mic_gain()) != ESP_OK) {
        muse_state_set_mode(MUSE_MODE_ERROR);
        muse_state_set_caption("AUDIO INIT FAILED");
        return ESP_FAIL;
    }
    muse_voice_reserve_note_buffer();   /* fallback if app startup did not take it */
    /* Create the task FIRST, while internal heap is still ~10K (before
     * wake init consumes it): the task's TCB needs internal RAM and
     * would not fit after wake init leaves only ~100-200 bytes free.
     * Stack in PSRAM if there is any (this task never writes flash)
     * to spare internal RAM for Wi-Fi/BLE. */
    if (xTaskCreatePinnedToCoreWithCaps(voice_task, "muse_voice", 6144, NULL, 6, NULL, MUSE_AUDIO_CORE,
                                        MUSE_BIG_CAPS) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    /* Waking is optional: without it (no model picked, no partition, or
     * the feature off) BOOT still works, so its failure never fails boot.
     * It runs after task creation, so its heap use never blocks the
     * task; a task already running feeds the AFE only once it is
     * ready (feed returns false until then). */
    (void)muse_wake_init();
    return ESP_OK;
}

void muse_voice_set_monitor(bool on)
{
    s_monitor_db = -100.0f;
    s_monitor = on;
}

float muse_voice_monitor_db(void)
{
    return s_monitor_db;
}

void muse_voice_request_chirp(void)
{
    s_chirp = true;
    muse_state_nudge();
}

void muse_voice_request_snore(void)
{
    uint32_t ms = now_ms();
    s_snore_ms = ms ? ms : 1;
}

void muse_voice_request_loopback(void)
{
    s_loopback = true;
    muse_state_nudge();
}

void muse_voice_request_mp3test(void)
{
    s_mp3test = true;
    muse_state_nudge();
}

void muse_voice_request_cancel(void)
{
    s_cancel_requested = true;
}

bool muse_voice_in_call(void)
{
    return s_call;
}

void muse_voice_request_hangup(void)
{
    s_hangup_requested = true;
}

/* Seconds recorded so far in the current note (0 when idle), for the
 * cancel button's timer. MAX_SECS lives here, so the UI does not
 * duplicate it. */
float muse_voice_record_secs(void)
{
    return muse_state_progress() * MAX_SECS;
}

bool muse_voice_resting(void)
{
    return s_resting;
}

bool muse_voice_notes_waiting(void)
{
    return s_waiting;
}
