/* Copyright (c) Meta Platforms, Inc. and affiliates. */
/*
 * WALL-E, the pixel avatar for this board.
 *
 * What he looks like: the Pixar trash-compactor robot. A boxy mustard-yellow
 * torso with a darker outline, a compactor-door seam, a chest door and a few
 * rust spots, riding on two dark caterpillar treads. A thin grey neck stalk
 * rises from the torso to his head: two tilted binocular barrels (not
 * round eyeballs), high at the bridge and sloping down outward,
 * silver-grey housings with dark camera lenses, joined by a small bridge
 * bar at their high inner ends. Two articulated arms hang at the torso's
 * sides: yellow upper arms, grey forearms, dark pincer claws. The lower
 * chest carries his name in a 3x5 pixel stencil: WALL-E.
 * Colours: mustard yellow body, dark brown outline and rust, dark grey
 * treads, silver-grey neck and eye housings, dark camera lenses with a
 * grey centre and a white catchlight.
 * Personality: curious and shy. He bobs gently when idle, his eyes widen
 * while listening, he tilts his head and glances around while thinking,
 * his eyes brighten while speaking, and he bounces when happy. Sad and
 * droopy on error, eyes shut and dimmed when off, and he snores Zzz
 * while falling asleep, and hearts float above his head when he is
 * happy. His eyes do all the talking: he has no mouth.
 * Reactions dizzy, waking and tickle are ignored on purpose: he still
 * renders, just without those extra animations. Sleepy is not ignored:
 * his eyes droop shut and Zzz floats up while he falls asleep.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "muse_pixel.h"

#define W MUSE_PX_W
#define H MUSE_PX_H

#define RGB565(r, g, b) (uint16_t)((uint16_t)((r) >> 3) << 11 | (uint16_t)((g) >> 2) << 5 | (uint16_t)((b) >> 3))

#define C_BG RGB565(0x00, 0x00, 0x00)
#define C_BODY RGB565(0xD9, 0xA4, 0x1C)
#define C_BODY_DARK RGB565(0xB0, 0x8A, 0x18)
#define C_OUTLINE RGB565(0x6B, 0x54, 0x10)
#define C_RUST RGB565(0x7A, 0x4A, 0x1F)
#define C_TREAD RGB565(0x2B, 0x2E, 0x33)
#define C_TREAD_MARK RGB565(0x55, 0x59, 0x5E)
#define C_NECK RGB565(0xB8, 0xBE, 0xC4)
#define C_HOUSING RGB565(0xB8, 0xBE, 0xC4)
#define C_HOUSING_SHADE RGB565(0x8A, 0x90, 0x96)
#define C_LENS RGB565(0x1A, 0x25, 0x30)
#define C_RIM RGB565(0x3A, 0x3E, 0x42)
#define C_LENS_INNER RGB565(0x5A, 0x60, 0x66)
#define C_LENS_INNER_BRIGHT RGB565(0x8A, 0x90, 0x96)
#define C_CATCH RGB565(0xFF, 0xFF, 0xFF)
#define C_HEART RGB565(0xFF, 0x55, 0x55)
#define C_ARM_GREY RGB565(0x8A, 0x90, 0x96)
#define C_CLAW RGB565(0x3A, 0x3E, 0x42)
#define C_DIM RGB565(0x5A, 0x5E, 0x60)

static uint16_t s_fb[W * H];
static int s_size = W;

static void set_px(int x, int y, uint16_t c)
{
    if (x >= 0 && x < W && y >= 0 && y < H) {
        s_fb[y * W + x] = c;
    }
}

static int iround(float v)
{
    return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Fill a rotated ellipse (angle in radians, long axis rotated from horizontal). */
static void fill_rot_ellipse(float cx, float cy, float rx, float ry, float ang, uint16_t c)
{
    float ca = cosf(ang);
    float sa = sinf(ang);
    float ex = sqrtf((rx * ca) * (rx * ca) + (ry * sa) * (ry * sa));
    float ey = sqrtf((rx * sa) * (rx * sa) + (ry * ca) * (ry * ca));
    int x0 = iround(cx - ex);
    int x1 = iround(cx + ex);
    int y0 = iround(cy - ey);
    int y1 = iround(cy + ey);
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            float dx = (float)x - cx;
            float dy = (float)y - cy;
            float lx = dx * ca + dy * sa;
            float ly = -dx * sa + dy * ca;
            float v = (lx * lx) / (rx * rx) + (ly * ly) / (ry * ry);
            if (v <= 1.0f) {
                set_px(x, y, c);
            }
        }
    }
}

/* Fill an axis-aligned rectangle, inclusive. */
static void fill_rect(int x0, int y0, int x1, int y1, uint16_t c)
{
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            set_px(x, y, c);
        }
    }
}

/* Draw a 1 px line (Bresenham). */
static void draw_line(int x0, int y0, int x1, int y1, uint16_t c)
{
    int dx = x1 - x0;
    int sx = dx >= 0 ? 1 : -1;
    dx = dx >= 0 ? dx : -dx;
    int dy = y1 - y0;
    int sy = dy >= 0 ? 1 : -1;
    dy = dy >= 0 ? dy : -dy;
    int err = dx - dy;
    int x = x0;
    int y = y0;
    while (true) {
        set_px(x, y, c);
        if (x == x1 && y == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

/* 3x5 pixel font for the "WALL-E" chest label. Each row is 3 bits,
 * bit 2 = left column. */
static const uint8_t FONT_W[5] = { 5, 5, 5, 7, 2 };
static const uint8_t FONT_A[5] = { 2, 5, 7, 5, 5 };
static const uint8_t FONT_L[5] = { 4, 4, 4, 4, 7 };
static const uint8_t FONT_DASH[5] = { 0, 0, 7, 0, 0 };
static const uint8_t FONT_E[5] = { 7, 4, 7, 4, 7 };

static void draw_glyph(int x0, int y0, const uint8_t *g, uint16_t c)
{
    for (int r = 0; r < 5; r++) {
        for (int col = 0; col < 3; col++) {
            if (g[r] & (1 << (2 - col))) {
                set_px(x0 + col, y0 + r, c);
            }
        }
    }
}

/* Z for snoring: full bars top and bottom, diagonal between. */
static const uint8_t FONT_Z[5] = { 7, 1, 2, 4, 7 };

/* draw_glyph with each pixel blown up to scale x scale. */
static void draw_glyph_scaled(int x0, int y0, const uint8_t *g, int scale, uint16_t c)
{
    for (int r = 0; r < 5; r++) {
        for (int col = 0; col < 3; col++) {
            if (g[r] & (1 << (2 - col))) {
                fill_rect(x0 + col * scale, y0 + r * scale,
                          x0 + col * scale + scale - 1, y0 + r * scale + scale - 1, c);
            }
        }
    }
}

/* 5x5 pixel heart, bit 4 = left column. */
static const uint8_t HEART[5] = { 10, 31, 31, 14, 4 };

static void draw_heart(int x0, int y0, uint16_t c)
{
    for (int r = 0; r < 5; r++) {
        for (int col = 0; col < 5; col++) {
            if (HEART[r] & (1 << (4 - col))) {
                set_px(x0 + col, y0 + r, c);
            }
        }
    }
}

static void draw_wall_e_label(int x0, int y0, uint16_t c)
{
    draw_glyph(x0, y0, FONT_W, c);
    draw_glyph(x0 + 4, y0, FONT_A, c);
    draw_glyph(x0 + 8, y0, FONT_L, c);
    draw_glyph(x0 + 12, y0, FONT_L, c);
    draw_glyph(x0 + 16, y0, FONT_DASH, c);
    draw_glyph(x0 + 20, y0, FONT_E, c);
}

static float s_next_blink = 2.0f;
static float s_blink_start = -10.0f;
static uint32_t s_rng = 0x12345678u;

static float frand(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return (float)((s_rng >> 8) & 0xFFFF) / 65535.0f;
}

uint32_t muse_pixel_accent(muse_mode_t mode)
{
    switch (mode) {
        case MUSE_MODE_LISTENING:
            return 0xF0C020;
        case MUSE_MODE_THINKING:
            return 0xE8A020;
        case MUSE_MODE_SPEAKING:
            return 0xD9A41C;
        case MUSE_MODE_ERROR:
            return 0xFF3B30;
        case MUSE_MODE_OFF:
            return 0x404040;
        default:
            return 0xD9A41C;
    }
}

void muse_pixel_set_size(int px)
{
    if (px < 1) {
        px = 1;
    }
    if (px > 512) {
        px = 512;
    }
    s_size = px;
}

void muse_pixel_scale(uint16_t *dst, int stride_px, int x0, int x1, int y0, int y1)
{
    for (int y = y0; y <= y1; y++) {
        int gy = y * H / s_size;
        if (gy < 0) {
            gy = 0;
        }
        if (gy >= H) {
            gy = H - 1;
        }
        uint16_t *out = dst + (y - y0) * stride_px;
        for (int x = x0; x <= x1; x++) {
            int gx = x * W / s_size;
            if (gx < 0) {
                gx = 0;
            }
            if (gx >= W) {
                gx = W - 1;
            }
            out[x - x0] = s_fb[gy * W + gx];
        }
    }
}

void muse_pixel_render(const muse_pose_t *p)
{
    for (int i = 0; i < W * H; i++) {
        s_fb[i] = C_BG;
    }

    float t = p->t;

    /* Blink: the shutters close and reopen over 0.14 s, every 2-4 s. */
    if (t >= s_next_blink) {
        s_blink_start = t;
        s_next_blink = t + 2.0f + frand() * 2.0f;
    }
    float bt = t - s_blink_start;
    float eye_open = 1.0f;
    if (bt >= 0.0f && bt < 0.14f) {
        float k = bt < 0.07f ? 1.0f - bt / 0.07f : (bt - 0.07f) / 0.07f;
        eye_open = k < 0.15f ? 0.15f : k;
    }

    /* Gentle idle bob, a bounce while happy, a faster bob while speaking. */
    float bob = sinf(t * 2.0f) * 1.0f;
    if (p->happy > 0.0f) {
        bob += sinf(t * 8.0f) * p->happy * 2.0f;
    }
    float lvl = clampf(p->level, 0.0f, 1.0f);
    if (p->mode == MUSE_MODE_SPEAKING) {
        bob += sinf(t * 10.0f) * lvl * 1.0f;
    }
    /* Falling asleep: the bob settles as sleepy rises. */
    float sleepy = clampf(p->sleepy, 0.0f, 1.0f);
    if (sleepy > 0.0f) {
        bob *= 1.0f - 0.7f * sleepy;
    }
    int yoff = iround(bob);
    if (yoff < -2) {
        yoff = -2;
    }
    if (yoff > 2) {
        yoff = 2;
    }

    /* Eye pose: gaze while thinking, widen while listening, droop on error,
     * look up when happy, rise while booting. */
    float gaze_x = 0.0f;
    float tilt = 0.0f;
    float ey_shift = 0.0f;
    float eye_wide = 1.0f;
    if (p->mode == MUSE_MODE_THINKING) {
        gaze_x = sinf(t * 3.0f) * 2.0f;
        tilt = 1.0f;
    }
    if (p->mode == MUSE_MODE_LISTENING) {
        eye_wide = 1.15f;
    }
    if (p->mode == MUSE_MODE_ERROR) {
        ey_shift = 2.0f;
        if (eye_open > 0.6f) {
            eye_open = 0.6f;
        }
    }
    if (p->happy > 0.0f) {
        ey_shift -= 1.0f;
    }
    /* Falling asleep: eyes droop and shut as sleepy rises. */
    if (sleepy > 0.0f) {
        ey_shift += 2.0f * sleepy;
        float shut = 1.0f - 0.85f * sleepy;
        if (eye_open > shut) {
            eye_open = shut;
        }
    }
    /* Happy: eyes widen and brighten. */
    float happy = clampf(p->happy, 0.0f, 1.0f);
    if (happy > 0.0f) {
        eye_wide += 0.25f * happy;
    }
    float boot_open = 1.0f;
    if (p->mode == MUSE_MODE_BOOT) {
        boot_open = clampf(p->mode_t / 0.5f, 0.0f, 1.0f);
        ey_shift += (1.0f - boot_open) * 4.0f;
    }

    bool off = (p->mode == MUSE_MODE_OFF);
    float ey_l = 15.0f + (float)yoff + ey_shift + tilt;
    float ey_r = 15.0f + (float)yoff + ey_shift - tilt;
    float ex_l = 23.0f + gaze_x;
    float ex_r = 41.0f + gaze_x;

    /* Treads. */
    uint16_t tread_c = off ? C_DIM : C_TREAD;
    fill_rect(10, 52 + yoff, 26, 61 + yoff, tread_c);
    fill_rect(38, 52 + yoff, 54, 61 + yoff, tread_c);
    if (!off) {
        fill_rect(14, 54 + yoff, 14, 59 + yoff, C_TREAD_MARK);
        fill_rect(18, 54 + yoff, 18, 59 + yoff, C_TREAD_MARK);
        fill_rect(22, 54 + yoff, 22, 59 + yoff, C_TREAD_MARK);
        fill_rect(42, 54 + yoff, 42, 59 + yoff, C_TREAD_MARK);
        fill_rect(46, 54 + yoff, 46, 59 + yoff, C_TREAD_MARK);
        fill_rect(50, 54 + yoff, 50, 59 + yoff, C_TREAD_MARK);
    }

    /* Boxy torso: dark outline, yellow inside, seam, chest door, rust. */
    uint16_t body_c = off ? C_DIM : C_BODY;
    fill_rect(15, 29 + yoff, 49, 52 + yoff, off ? C_DIM : C_OUTLINE);
    fill_rect(16, 30 + yoff, 48, 51 + yoff, body_c);
    if (!off) {
        draw_wall_e_label(21, 46 + yoff, C_OUTLINE);
        fill_rect(17, 40 + yoff, 47, 40 + yoff, C_OUTLINE);
        fill_rect(25, 36 + yoff, 39, 44 + yoff, C_BODY_DARK);
        set_px(18, 48 + yoff, C_RUST);
        set_px(45, 48 + yoff, C_RUST);
        set_px(20, 33 + yoff, C_RUST);
    }

    /* Neck stalk up to the bridge between the eyes. */
    fill_rect(30, 10 + yoff, 34, 30 + yoff, off ? C_DIM : C_NECK);

    /* Binocular barrels, not round eyeballs: tilted housings, high at the
     * bridge and sloping down outward (left /, right \), dark lenses at
     * their ends, joined by a bridge bar at the high inner ends where the
     * neck meets them. The housing keeps its original size; only the lens
 * (the eyeball) is a little smaller. */
    uint16_t house_c = off ? C_DIM : C_HOUSING;
    fill_rot_ellipse(ex_l, ey_l, 9.0f * eye_wide, 6.0f * eye_wide, -0.524f, house_c);
    fill_rot_ellipse(ex_r, ey_r, 9.0f * eye_wide, 6.0f * eye_wide, 0.524f, house_c);
    if (!off) {
        int bar_y = iround((ey_l + ey_r) / 2.0f) - 4;
        fill_rect(29, bar_y - 1, 35, bar_y + 1, C_NECK);
    }
    if (!off) {
        float lry = 3.2f * eye_open * boot_open * eye_wide;
        float iry = 2.0f * eye_open * boot_open * eye_wide;
        float rry = 5.0f * eye_open * boot_open * eye_wide;
        if (lry < 0.5f) {
            lry = 0.5f;
        }
        if (iry < 0.5f) {
            iry = 0.5f;
        }
        if (rry < 0.5f) {
            rry = 0.5f;
        }
        float lrx = 5.0f * eye_wide;
        float irx = 3.0f * eye_wide;
        float rrx = 7.5f * eye_wide;
        fill_rot_ellipse(ex_l, ey_l, rrx, rry, -0.524f, C_RIM);
        fill_rot_ellipse(ex_r, ey_r, rrx, rry, 0.524f, C_RIM);
        fill_rot_ellipse(ex_l, ey_l, lrx, lry, -0.524f, C_LENS);
        fill_rot_ellipse(ex_r, ey_r, lrx, lry, 0.524f, C_LENS);
        uint16_t inner_c = C_LENS_INNER;
        if ((p->mode == MUSE_MODE_SPEAKING && lvl > 0.5f) || happy > 0.3f) {
            inner_c = C_LENS_INNER_BRIGHT;
        }
        fill_rot_ellipse(ex_l, ey_l, irx, iry, -0.524f, inner_c);
        fill_rot_ellipse(ex_r, ey_r, irx, iry, 0.524f, inner_c);
        if (eye_open > 0.5f) {
            set_px(iround(ex_l) - 2, iround(ey_l) - 2, C_CATCH);
            set_px(iround(ex_r) - 2, iround(ey_r) - 2, C_CATCH);
        }
    }

    /* Arms at the torso's sides: yellow upper, grey forearm, dark pincer. */
    uint16_t up_c = off ? C_DIM : C_BODY;
    uint16_t fore_c = off ? C_DIM : C_ARM_GREY;
    uint16_t claw_c = off ? C_DIM : C_CLAW;
    fill_rot_ellipse(13.5f, 37.0f + (float)yoff, 4.7f, 2.0f, -1.012f, up_c);
    fill_rot_ellipse(50.5f, 37.0f + (float)yoff, 4.7f, 2.0f, 1.012f, up_c);
    fill_rot_ellipse(10.0f, 44.0f + (float)yoff, 3.2f, 2.0f, -1.257f, fore_c);
    fill_rot_ellipse(54.0f, 44.0f + (float)yoff, 3.2f, 2.0f, 1.257f, fore_c);
    draw_line(9, 47 + yoff, 6, 50 + yoff, claw_c);
    draw_line(9, 47 + yoff, 11, 50 + yoff, claw_c);
    draw_line(55, 47 + yoff, 52, 50 + yoff, claw_c);
    draw_line(55, 47 + yoff, 58, 50 + yoff, claw_c);

    /* Snore: Zzz floats up to the right of the head while falling asleep,
     * one letter at a time as sleepy rises, bobbing gently. Not when off:
     * the screen is dark by then anyway. */
    if (!off && sleepy > 0.0f) {
        if (sleepy > 0.15f) {
            draw_glyph(50, 12 + iround(sinf(t * 2.0f) * 1.0f), FONT_Z, C_CATCH);
        }
        if (sleepy > 0.45f) {
            draw_glyph(55, 7 + iround(sinf(t * 2.0f + 2.1f) * 1.0f), FONT_Z, C_CATCH);
        }
        if (sleepy > 0.7f) {
            int z3y = iround(sinf(t * 2.0f + 4.2f) * 1.0f);
            if (z3y < 0) {
                z3y = 0;
            }
            draw_glyph_scaled(56, z3y, FONT_Z, 2, C_CATCH);
        }
    }

    /* Happy: hearts float above the head, like the default avatar. */
    if (!off && happy > 0.2f) {
        draw_heart(10, 3 + iround(sinf(t * 3.0f) * 1.0f), C_HEART);
        if (happy > 0.6f) {
            draw_heart(20, 1 + iround(sinf(t * 3.0f + 2.1f) * 1.0f), C_HEART);
        }
    }
}
