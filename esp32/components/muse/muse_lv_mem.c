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
 * LVGL allocator: widgets and styles live in PSRAM. With the C library malloc,
 * ESP-IDF puts every allocation under 16 KB in internal RAM, and a few hundred
 * settings widgets would starve Wi-Fi and the display's DMA bounce buffers.
 */
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "lvgl.h"
#include "src/stdlib/lv_mem.h"

#define LV_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

/* Measurement (v3.1): best-effort LVGL allocator counters. The allocator
 * is a passthrough to heap_caps (PSRAM first, plain malloc fallback), so
 * there is no pool to walk; live bytes/count come from these counters
 * (atomic: LVGL calls may come from more than one task), peak tracks
 * the worst case. Block sizes come from heap_caps_get_allocated_size,
 * which works for heap_caps and plain malloc pointers alike on ESP-IDF
 * (malloc routes through the same multi-heap). Sizes are requested
 * sizes when the allocated size is unavailable; counts stay exact. */
static long s_lv_live_bytes;
static long s_lv_peak_bytes;
static long s_lv_live_count;

static void lv_note_live(long live)
{
    long peak = __atomic_load_n(&s_lv_peak_bytes, __ATOMIC_RELAXED);
    while (live > peak &&
           !__atomic_compare_exchange_n(&s_lv_peak_bytes, &peak, live, 0,
                                        __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {
    }
}

static size_t lv_block_size(void *p, size_t fallback)
{
    size_t size = p ? heap_caps_get_allocated_size(p) : 0;
    return size ? size : fallback;
}

static void lv_track_alloc(void *p, size_t requested)
{
    if (!p) {
        return;
    }
    long live = __atomic_add_fetch(&s_lv_live_bytes, (long)lv_block_size(p, requested),
                                   __ATOMIC_RELAXED);
    __atomic_add_fetch(&s_lv_live_count, 1, __ATOMIC_RELAXED);
    lv_note_live(live);
}

static void lv_track_free(void *p)
{
    if (!p) {
        return;
    }
    __atomic_sub_fetch(&s_lv_live_bytes, (long)lv_block_size(p, 0), __ATOMIC_RELAXED);
    __atomic_sub_fetch(&s_lv_live_count, 1, __ATOMIC_RELAXED);
}

void lv_mem_init(void)
{
}

void lv_mem_deinit(void)
{
}

lv_mem_pool_t lv_mem_add_pool(void *mem, size_t bytes)
{
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    LV_UNUSED(pool);
}

void *lv_malloc_core(size_t size)
{
    void *p = heap_caps_malloc(size, LV_CAPS);
    if (!p) {
        p = malloc(size);
    }
    lv_track_alloc(p, size);
    return p;
}

void *lv_realloc_core(void *p, size_t new_size)
{
    size_t old = lv_block_size(p, 0);
    void *q = heap_caps_realloc(p, new_size, LV_CAPS);
    if (!q) {
        q = realloc(p, new_size);
    }
    if (q) {
        if (!p) {
            __atomic_add_fetch(&s_lv_live_count, 1, __ATOMIC_RELAXED);
        }
        long live = __atomic_add_fetch(&s_lv_live_bytes,
                                       (long)lv_block_size(q, new_size) - (long)old,
                                       __ATOMIC_RELAXED);
        lv_note_live(live);
    }
    return q;
}

void lv_free_core(void *p)
{
    lv_track_free(p);
    heap_caps_free(p);
}

void lv_mem_monitor_core(lv_mem_monitor_t *mon_p)
{
    if (!mon_p) {
        return;
    }
    /* The pool is shared PSRAM, not LVGL-exclusive: total/free/biggest
     * describe PSRAM, while used_cnt/max_used describe LVGL's share.
     * used_pct is LVGL live bytes over PSRAM total. free_cnt is
     * unavailable without a pool to walk. */
    uint32_t total = (uint32_t)heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    uint32_t free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    uint32_t biggest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    long live = __atomic_load_n(&s_lv_live_bytes, __ATOMIC_RELAXED);
    if (live < 0) {
        live = 0;
    }
    mon_p->total_size = total;
    mon_p->free_cnt = 0;
    mon_p->free_size = free;
    mon_p->free_biggest_size = biggest;
    mon_p->used_cnt = (uint32_t)__atomic_load_n(&s_lv_live_count, __ATOMIC_RELAXED);
    mon_p->max_used = (uint32_t)__atomic_load_n(&s_lv_peak_bytes, __ATOMIC_RELAXED);
    mon_p->used_pct = total ? (uint8_t)((uint64_t)live * 100 / total) : 0;
    mon_p->frag_pct = free ? (uint8_t)(100 - (uint64_t)biggest * 100 / free) : 0;
}

lv_result_t lv_mem_test_core(void)
{
    return LV_RESULT_OK;
}
