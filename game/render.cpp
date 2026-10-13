/**
  @file   render.cpp
  @author apfxtech
  @brief  1bpp framebuffer drawing on top of the Flipper canvas

  MicroTD - Tower Defense Game, Flipper Zero port

  apfxtech, 2026, license: MIT (see LICENSE)

  SPDX-License-Identifier: MIT
*/

#include "render.h"

#include <furi.h>
#include <string.h>

static Canvas* gfx_canvas = NULL;
static uint8_t* gfx_fb = NULL;

void gfx_bind(Canvas* canvas) {
    gfx_canvas = canvas;
    gfx_fb = canvas_get_buffer(canvas);
    canvas_set_font(canvas, FontSecondary);
}

void gfx_unbind(void) {
    gfx_canvas = NULL;
    gfx_fb = NULL;
}

static inline void gfx_apply(int16_t x, int16_t y, uint8_t value, uint8_t mask) {
    if(mask == 0 || (uint16_t)x >= GFX_WIDTH) return;

    const int16_t row = (int16_t)(y >> 3);
    const uint8_t shift = (uint8_t)(y & 7);

    if(shift == 0) {
        if((uint16_t)row < GFX_PAGES) {
            uint8_t* dst = &gfx_fb[row * GFX_WIDTH + x];
            *dst = (uint8_t)((*dst & (uint8_t)~mask) | (value & mask));
        }
        return;
    }

    const uint8_t low_mask = (uint8_t)(mask << shift);
    if(low_mask && (uint16_t)row < GFX_PAGES) {
        const uint8_t low_value = (uint8_t)(value << shift);
        uint8_t* dst = &gfx_fb[row * GFX_WIDTH + x];
        *dst = (uint8_t)((*dst & (uint8_t)~low_mask) | (low_value & low_mask));
    }

    const uint8_t high_mask = (uint8_t)(mask >> (8 - shift));
    const int16_t next_row = (int16_t)(row + 1);
    if(high_mask && (uint16_t)next_row < GFX_PAGES) {
        const uint8_t high_value = (uint8_t)(value >> (8 - shift));
        uint8_t* dst = &gfx_fb[next_row * GFX_WIDTH + x];
        *dst = (uint8_t)((*dst & (uint8_t)~high_mask) | (high_value & high_mask));
    }
}

void gfx_clear(GfxColor color) {
    memset(gfx_fb, color == GfxDark ? 0xFF : 0x00, GFX_SIZE);
}

void gfx_pixel(int16_t x, int16_t y, GfxColor color) {
    if((uint16_t)x >= GFX_WIDTH || (uint16_t)y >= GFX_HEIGHT) return;

    uint8_t* dst = &gfx_fb[((y >> 3) * GFX_WIDTH) + x];
    const uint8_t bit = (uint8_t)(1u << (y & 7));

    if(color == GfxDark) {
        *dst |= bit;
    } else {
        *dst &= (uint8_t)~bit;
    }
}

void gfx_vline(int16_t x, int16_t y, int16_t h, GfxColor color) {
    if(h <= 0 || (uint16_t)x >= GFX_WIDTH) return;

    int16_t y0 = y;
    int16_t y1 = (int16_t)(y + h);
    if(y0 < 0) y0 = 0;
    if(y1 > GFX_HEIGHT) y1 = GFX_HEIGHT;
    if(y0 >= y1) return;

    const uint8_t value = (color == GfxDark) ? 0xFF : 0x00;

    while(y0 < y1) {
        const uint8_t shift = (uint8_t)(y0 & 7);
        const int16_t page_end = (int16_t)(y0 - shift + 8);
        const int16_t span_end = (page_end < y1) ? page_end : y1;
        const uint8_t span = (uint8_t)(span_end - y0);
        const uint8_t mask =
            (uint8_t)(((span >= 8) ? 0xFF : (uint8_t)((1u << span) - 1)) << shift);

        uint8_t* dst = &gfx_fb[((y0 >> 3) * GFX_WIDTH) + x];
        *dst = (uint8_t)((*dst & (uint8_t)~mask) | (value & mask));

        y0 = span_end;
    }
}

void gfx_hline(int16_t x, int16_t y, int16_t w, GfxColor color) {
    if(w <= 0 || (uint16_t)y >= GFX_HEIGHT) return;

    int16_t x0 = x;
    int16_t x1 = (int16_t)(x + w);
    if(x0 < 0) x0 = 0;
    if(x1 > GFX_WIDTH) x1 = GFX_WIDTH;
    if(x0 >= x1) return;

    uint8_t* dst = &gfx_fb[((y >> 3) * GFX_WIDTH) + x0];
    const uint8_t bit = (uint8_t)(1u << (y & 7));

    if(color == GfxDark) {
        for(int16_t i = x0; i < x1; i++)
            *dst++ |= bit;
    } else {
        for(int16_t i = x0; i < x1; i++)
            *dst++ &= (uint8_t)~bit;
    }
}

void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, GfxColor color) {
    int16_t dx = (int16_t)(x1 > x0 ? x1 - x0 : x0 - x1);
    int16_t dy = (int16_t) - (y1 > y0 ? y1 - y0 : y0 - y1);
    int16_t step_x = (int16_t)(x0 < x1 ? 1 : -1);
    int16_t step_y = (int16_t)(y0 < y1 ? 1 : -1);
    int16_t err = (int16_t)(dx + dy);

    for(;;) {
        gfx_pixel(x0, y0, color);
        if(x0 == x1 && y0 == y1) break;

        const int16_t err2 = (int16_t)(2 * err);
        if(err2 >= dy) {
            err = (int16_t)(err + dy);
            x0 = (int16_t)(x0 + step_x);
        }
        if(err2 <= dx) {
            err = (int16_t)(err + dx);
            y0 = (int16_t)(y0 + step_y);
        }
    }
}

void gfx_rect(int16_t x, int16_t y, int16_t w, int16_t h, GfxColor color) {
    if(w <= 0 || h <= 0) return;

    gfx_hline(x, y, w, color);
    gfx_hline(x, (int16_t)(y + h - 1), w, color);
    gfx_vline(x, y, h, color);
    gfx_vline((int16_t)(x + w - 1), y, h, color);
}

void gfx_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, GfxColor color) {
    if(w <= 0 || h <= 0) return;

    int16_t x0 = x;
    int16_t x1 = (int16_t)(x + w);
    if(x0 < 0) x0 = 0;
    if(x1 > GFX_WIDTH) x1 = GFX_WIDTH;

    for(int16_t xx = x0; xx < x1; xx++) {
        gfx_vline(xx, y, h, color);
    }
}

void gfx_circle(int16_t x0, int16_t y0, int16_t r, GfxColor color) {
    int16_t f = (int16_t)(1 - r);
    int16_t ddf_x = 1;
    int16_t ddf_y = (int16_t)(-2 * r);
    int16_t x = 0;
    int16_t y = r;

    gfx_pixel(x0, (int16_t)(y0 + r), color);
    gfx_pixel(x0, (int16_t)(y0 - r), color);
    gfx_pixel((int16_t)(x0 + r), y0, color);
    gfx_pixel((int16_t)(x0 - r), y0, color);

    while(x < y) {
        if(f >= 0) {
            y--;
            ddf_y = (int16_t)(ddf_y + 2);
            f = (int16_t)(f + ddf_y);
        }

        x++;
        ddf_x = (int16_t)(ddf_x + 2);
        f = (int16_t)(f + ddf_x);

        gfx_pixel((int16_t)(x0 + x), (int16_t)(y0 + y), color);
        gfx_pixel((int16_t)(x0 - x), (int16_t)(y0 + y), color);
        gfx_pixel((int16_t)(x0 + x), (int16_t)(y0 - y), color);
        gfx_pixel((int16_t)(x0 - x), (int16_t)(y0 - y), color);
        gfx_pixel((int16_t)(x0 + y), (int16_t)(y0 + x), color);
        gfx_pixel((int16_t)(x0 - y), (int16_t)(y0 + x), color);
        gfx_pixel((int16_t)(x0 + y), (int16_t)(y0 - x), color);
        gfx_pixel((int16_t)(x0 - y), (int16_t)(y0 - x), color);
    }
}

void gfx_disc(int16_t x0, int16_t y0, int16_t r, GfxColor color) {
    int16_t f = (int16_t)(1 - r);
    int16_t ddf_x = 1;
    int16_t ddf_y = (int16_t)(-2 * r);
    int16_t x = 0;
    int16_t y = r;

    gfx_vline(x0, (int16_t)(y0 - r), (int16_t)(2 * r + 1), color);

    while(x < y) {
        if(f >= 0) {
            y--;
            ddf_y = (int16_t)(ddf_y + 2);
            f = (int16_t)(f + ddf_y);
        }

        x++;
        ddf_x = (int16_t)(ddf_x + 2);
        f = (int16_t)(f + ddf_x);

        gfx_vline((int16_t)(x0 + x), (int16_t)(y0 - y), (int16_t)(2 * y + 1), color);
        gfx_vline((int16_t)(x0 - x), (int16_t)(y0 - y), (int16_t)(2 * y + 1), color);
        gfx_vline((int16_t)(x0 + y), (int16_t)(y0 - x), (int16_t)(2 * x + 1), color);
        gfx_vline((int16_t)(x0 - y), (int16_t)(y0 - x), (int16_t)(2 * x + 1), color);
    }
}

static inline uint8_t gfx_page_mask(int16_t page, int16_t pages, int16_t h) {
    if(page + 1 < pages) return 0xFF;
    const uint8_t rest = (uint8_t)(h & 7);
    return rest ? (uint8_t)((1u << rest) - 1) : 0xFF;
}

void gfx_bitmap(int16_t x, int16_t y, const uint8_t* data, int16_t w, int16_t h, GfxColor color) {
    if(!data || w <= 0 || h <= 0) return;
    if(x + w <= 0 || x >= GFX_WIDTH || y + h <= 0 || y >= GFX_HEIGHT) return;

    const int16_t pages = (int16_t)((h + 7) >> 3);
    const uint8_t value = (color == GfxDark) ? 0xFF : 0x00;

    for(int16_t page = 0; page < pages; page++) {
        const uint8_t page_mask = gfx_page_mask(page, pages, h);
        const uint8_t* column = data + (int32_t)page * w;
        const int16_t page_y = (int16_t)(y + page * 8);

        for(int16_t c = 0; c < w; c++) {
            gfx_apply((int16_t)(x + c), page_y, value, (uint8_t)(column[c] & page_mask));
        }
    }
}

void gfx_sprite(int16_t x, int16_t y, const uint8_t* sheet, uint8_t frame) {
    if(!sheet) return;

    const int16_t w = (int16_t)sheet[0];
    const int16_t h = (int16_t)sheet[1];
    if(w <= 0 || h <= 0) return;
    if(x + w <= 0 || x >= GFX_WIDTH || y + h <= 0 || y >= GFX_HEIGHT) return;

    const int16_t pages = (int16_t)((h + 7) >> 3);
    const uint8_t* data = sheet + 2 + (int32_t)frame * w * pages;

    for(int16_t page = 0; page < pages; page++) {
        const uint8_t page_mask = gfx_page_mask(page, pages, h);
        const uint8_t* column = data + (int32_t)page * w;
        const int16_t page_y = (int16_t)(y + page * 8);

        for(int16_t c = 0; c < w; c++) {
            gfx_apply((int16_t)(x + c), page_y, (uint8_t)~column[c], page_mask);
        }
    }
}

void gfx_sprite_masked(int16_t x, int16_t y, const uint8_t* sheet, uint8_t frame) {
    if(!sheet) return;

    const int16_t w = (int16_t)sheet[0];
    const int16_t h = (int16_t)sheet[1];
    if(w <= 0 || h <= 0) return;
    if(x + w <= 0 || x >= GFX_WIDTH || y + h <= 0 || y >= GFX_HEIGHT) return;

    const int16_t pages = (int16_t)((h + 7) >> 3);
    const uint8_t* data = sheet + 2 + (int32_t)frame * w * pages * 2;

    for(int16_t page = 0; page < pages; page++) {
        const uint8_t page_mask = gfx_page_mask(page, pages, h);
        const uint8_t* column = data + (int32_t)page * w * 2;
        const int16_t page_y = (int16_t)(y + page * 8);

        for(int16_t c = 0; c < w; c++) {
            const uint8_t image = column[c * 2];
            const uint8_t mask = column[c * 2 + 1];
            gfx_apply((int16_t)(x + c), page_y, (uint8_t)~image, (uint8_t)(mask & page_mask));
        }
    }
}

int16_t gfx_text_width(const char* text) {
    if(!gfx_canvas || !text) return 0;
    return (int16_t)canvas_string_width(gfx_canvas, text);
}

void gfx_text(int16_t x, int16_t y, const char* text, GfxColor ink) {
    if(!gfx_canvas || !text) return;

    canvas_set_color(gfx_canvas, ink == GfxDark ? ColorBlack : ColorWhite);
    canvas_draw_str(gfx_canvas, x, (int16_t)(y + GFX_TEXT_BASELINE), text);
    canvas_set_color(gfx_canvas, ColorBlack);
}

void gfx_text_centered(int16_t center_x, int16_t y, const char* text, GfxColor ink) {
    gfx_text((int16_t)(center_x - gfx_text_width(text) / 2), y, text, ink);
}

void gfx_text_boxed(int16_t x, int16_t y, const char* text, GfxColor ink, GfxColor background) {
    if(!gfx_canvas || !text) return;

    gfx_fill_rect(x, y, (int16_t)(gfx_text_width(text) + 1), GFX_TEXT_HEIGHT, background);
    gfx_text(x, y, text, ink);
}
