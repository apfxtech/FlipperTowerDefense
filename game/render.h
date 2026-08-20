/**
  @file   render.h
  @author apfxtech
  @brief  1bpp framebuffer drawing on top of the Flipper canvas

  MicroTD - Tower Defense Game, Flipper Zero port

  apfxtech, 2026, license: MIT (see LICENSE)

  SPDX-License-Identifier: MIT
*/

#pragma once

#include <gui/gui.h>
#include <stdbool.h>
#include <stdint.h>

#define GFX_WIDTH  128
#define GFX_HEIGHT 64
#define GFX_PAGES  (GFX_HEIGHT / 8)
#define GFX_SIZE   (GFX_WIDTH * GFX_PAGES)

#define GFX_TEXT_HEIGHT   8
#define GFX_TEXT_BASELINE 7

typedef enum {
    GfxLight = 0,
    GfxDark = 1,
} GfxColor;

#ifdef __cplusplus
extern "C" {
#endif

void gfx_bind(Canvas* canvas);
void gfx_unbind(void);

void gfx_clear(GfxColor color);
void gfx_pixel(int16_t x, int16_t y, GfxColor color);
void gfx_hline(int16_t x, int16_t y, int16_t w, GfxColor color);
void gfx_vline(int16_t x, int16_t y, int16_t h, GfxColor color);
void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, GfxColor color);
void gfx_rect(int16_t x, int16_t y, int16_t w, int16_t h, GfxColor color);
void gfx_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, GfxColor color);
void gfx_circle(int16_t x0, int16_t y0, int16_t r, GfxColor color);
void gfx_disc(int16_t x0, int16_t y0, int16_t r, GfxColor color);

void gfx_bitmap(int16_t x, int16_t y, const uint8_t* data, int16_t w, int16_t h, GfxColor color);
void gfx_sprite(int16_t x, int16_t y, const uint8_t* sheet, uint8_t frame);
void gfx_sprite_masked(int16_t x, int16_t y, const uint8_t* sheet, uint8_t frame);

void gfx_text(int16_t x, int16_t y, const char* text, GfxColor ink);
void gfx_text_centered(int16_t center_x, int16_t y, const char* text, GfxColor ink);
void gfx_text_boxed(int16_t x, int16_t y, const char* text, GfxColor ink, GfxColor background);
int16_t gfx_text_width(const char* text);

#ifdef __cplusplus
}
#endif
