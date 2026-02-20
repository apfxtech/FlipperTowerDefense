#pragma once

#include "Arduino.h"
#include "ArduboyAudio.h"
#include "ArduboyTones.h"
#include "EEPROM.h"
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <furi.h>
#include <input/input.h>
#include <gui/gui.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#ifdef __cplusplus
}
#endif

#ifndef WIDTH
#define WIDTH 128
#endif
#ifndef HEIGHT
#define HEIGHT 64
#endif

#ifndef UP_BUTTON
#define UP_BUTTON    0x01
#define DOWN_BUTTON  0x02
#define LEFT_BUTTON  0x04
#define RIGHT_BUTTON 0x08
#define A_BUTTON     0x10
#define B_BUTTON     0x20
#endif

#ifndef INPUT_UP
#define INPUT_UP UP_BUTTON
#endif
#ifndef INPUT_DOWN
#define INPUT_DOWN DOWN_BUTTON
#endif
#ifndef INPUT_LEFT
#define INPUT_LEFT LEFT_BUTTON
#endif
#ifndef INPUT_RIGHT
#define INPUT_RIGHT RIGHT_BUTTON
#endif
#ifndef INPUT_A
#define INPUT_A A_BUTTON
#endif
#ifndef INPUT_B
#define INPUT_B B_BUTTON
#endif

#define ARDUBOY_LIB_VER               60000
#define ARDUBOY_UNIT_NAME_LEN         6
#define ARDUBOY_UNIT_NAME_BUFFER_SIZE (ARDUBOY_UNIT_NAME_LEN + 1)
#define EEPROM_STORAGE_SPACE_START    16
#define BLACK                         0
#define WHITE                         1
#define INVERT                        2
#define CLEAR_BUFFER                  true

struct Rect {
    int x;
    int y;
    int width;
    int height;

    Rect() = default;
    constexpr Rect(int x, int y, int width, int height)
        : x(x)
        , y(y)
        , width(width)
        , height(height) {
    }
};

struct Point {
    int x;
    int y;
    Point() = default;
    constexpr Point(int x, int y)
        : x(x)
        , y(y) {
    }
};

class Arduboy2Base {
public:
    ArduboyAudio audio;

    struct InputContext {
        volatile uint8_t* input_state = nullptr;
    };

    InputContext* inputContext() {
        return &input_ctx_;
    }

    void begin(
        uint8_t* screen_buffer,
        volatile uint8_t* input_state,
        FuriMutex* game_mutex,
        volatile bool* exit_requested) {
        sBuffer_ = screen_buffer;
        input_state_ = input_state;
        input_ctx_.input_state = input_state_;
        game_mutex_ = game_mutex;
        exit_requested_ = exit_requested;
        external_timing_ = false;
        frameCount = 0;
        last_frame_ms_ = 0;
    }

    void begin() {
    }

    void exitToBootloader() {
        if(exit_requested_) {
            *exit_requested_ = true;
        }
    }

    bool collide(Point point, Rect rect) {
        return (point.x >= rect.x) && (point.x < rect.x + rect.width) && (point.y >= rect.y) &&
               (point.y < rect.y + rect.height);
    }

    bool collide(Rect rect1, Rect rect2) {
        return !(
            rect2.x >= rect1.x + rect1.width || rect2.x + rect2.width <= rect1.x ||
            rect2.y >= rect1.y + rect1.height || rect2.y + rect2.height <= rect1.y);
    }

    static void FlipperInputCallback(const InputEvent* event, void* ctx_ptr) {
        if(!event || !ctx_ptr) return;
        InputContext* ctx = (InputContext*)ctx_ptr;
        volatile uint8_t* st = ctx->input_state;
        if(!st) return;

        uint8_t bit = 0;
        switch(event->key) {
        case InputKeyUp:
            bit = INPUT_UP;
            break;
        case InputKeyDown:
            bit = INPUT_DOWN;
            break;
        case InputKeyLeft:
            bit = INPUT_LEFT;
            break;
        case InputKeyRight:
            bit = INPUT_RIGHT;
            break;
        case InputKeyOk:
            bit = INPUT_A;
            break;
        case InputKeyBack:
            bit = INPUT_B;
            break;
        default:
            return;
        }

        if(event->type == InputTypePress) {
            (void)__atomic_fetch_or((uint8_t*)st, bit, __ATOMIC_RELAXED);
        } else if(event->type == InputTypeRelease) {
            (void)__atomic_fetch_and((uint8_t*)st, (uint8_t)~bit, __ATOMIC_RELAXED);
        }
    }

    void boot() {
    }
    void bootLogo() {
    }
    void bootLogoSpritesSelfMasked() {
    }

    void setFrameRate(uint8_t fps) {
        if(fps == 0) fps = 60;
        uint32_t d = 1000u / fps;
        frame_duration_ms_ = d ? d : 1u;
    }

    bool nextFrame() {
        if(external_timing_) {
            frameCount++;
            return true;
        }
        const uint32_t now = millis();
        if((uint32_t)(now - last_frame_ms_) < frame_duration_ms_) return false;
        last_frame_ms_ = now;
        frameCount++;
        return true;
    }

    bool everyXFrames(uint8_t n) const {
        if(n == 0) return false;
        return (frameCount % n) == 0;
    }

    void pollButtons() {
        prev_buttons_ = cur_buttons_;
        const uint8_t in =
            input_state_ ? __atomic_load_n((uint8_t*)input_state_, __ATOMIC_RELAXED) : 0;
        cur_buttons_ = mapInputToArduboyMask_(in);
    }

    void clearButtonState() {
        prev_buttons_ = 0;
        cur_buttons_ = 0;
    }

    bool pressed(uint8_t mask) const {
        return (cur_buttons_ & mask) != 0;
    }
    bool notPressed(uint8_t mask) const {
        return (cur_buttons_ & mask) == 0;
    }
    bool justPressed(uint8_t mask) const {
        return ((cur_buttons_ & mask) != 0) && ((prev_buttons_ & mask) == 0);
    }
    bool justReleased(uint8_t mask) const {
        return ((cur_buttons_ & mask) == 0) && ((prev_buttons_ & mask) != 0);
    }

    uint8_t* getBuffer() {
        return sBuffer_;
    }
    const uint8_t* getBuffer() const {
        return sBuffer_;
    }

    void clear() {
        fillScreen(0);
    }

    void fillScreen(uint8_t color) {
        if(!sBuffer_) return;
        memset(sBuffer_, color ? 0xFF : 0x00, (WIDTH * HEIGHT) / 8);
    }

    void display() {
    }

    void drawPixel(int16_t x, int16_t y, uint8_t color) {
        uint16_t ux = (uint16_t)x;
        uint16_t uy = (uint16_t)y;

        if(!sBuffer_ || ux >= WIDTH || uy >= HEIGHT) return;

        uint8_t* dst = sBuffer_ + ux + (uint16_t)((uy >> 3) * WIDTH);
        uint8_t mask = (uint8_t)(1u << (uy & 7));

        uint8_t v = *dst;
        v = (uint8_t)((v & (uint8_t)~mask) | ((uint8_t)-(uint8_t)(color != 0) & mask));
        *dst = v;
    }

    void drawBitmapFrame(int16_t x, int16_t y, const uint8_t* bmp, uint8_t frame) {
        if(!bmp || !sBuffer_) return;
        const int16_t w = (int16_t)pgm_read_byte(bmp + 0);
        const int16_t h = (int16_t)pgm_read_byte(bmp + 1);
        if(!isSpriteVisible_(x, y, w, h)) return;
        const int16_t pages = (h + 7) >> 3;
        const uint16_t frame_size = (uint16_t)w * (uint16_t)pages;
        const uint8_t* data = bmp + 2 + (uint32_t)frame * (uint32_t)frame_size;
        blitSelfMasked_(x, y, data, w, h);
    }

    void drawSolidBitmapFrame(int16_t x, int16_t y, const uint8_t* bmp, uint8_t frame) {
        if(!bmp || !sBuffer_) return;
        const int16_t w = (int16_t)pgm_read_byte(bmp + 0);
        const int16_t h = (int16_t)pgm_read_byte(bmp + 1);
        if(!isSpriteVisible_(x, y, w, h)) return;
        const int16_t pages = (h + 7) >> 3;
        const uint16_t frame_size = (uint16_t)w * (uint16_t)pages;
        const uint8_t* data = bmp + 2 + (uint32_t)frame * (uint32_t)frame_size;
        blitOverwrite_(x, y, data, w, h);
    }

    void eraseBitmapFrame(int16_t x, int16_t y, const uint8_t* bmp, uint8_t frame) {
        if(!bmp || !sBuffer_) return;
        const int16_t w = (int16_t)pgm_read_byte(bmp + 0);
        const int16_t h = (int16_t)pgm_read_byte(bmp + 1);
        if(!isSpriteVisible_(x, y, w, h)) return;
        const int16_t pages = (h + 7) >> 3;
        const uint16_t frame_size = (uint16_t)w * (uint16_t)pages;
        const uint8_t* data = bmp + 2 + (uint32_t)frame * (uint32_t)frame_size;
        blitErase_(x, y, data, w, h);
    }

    void drawPlusMask(int16_t x, int16_t y, const uint8_t* plusmask, uint8_t frame) {
        if(!plusmask || !sBuffer_) return;
        const int16_t w = (int16_t)pgm_read_byte(plusmask + 0);
        const int16_t h = (int16_t)pgm_read_byte(plusmask + 1);
        if(!isSpriteVisible_(x, y, w, h)) return;
        const int16_t pages = (h + 7) >> 3;
        const uint16_t frame_size = (uint16_t)w * (uint16_t)pages;
        const uint8_t* data = plusmask + 2 + (uint32_t)frame * (uint32_t)frame_size * 2u;
        blitPlusMask_(x, y, data, w, h);
    }

    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t color) {
        if(r <= 0) return;
        int16_t x = r;
        int16_t y = 0;
        int16_t err = 1 - x;

        while(x >= y) {
            drawPixel(x0 + x, y0 + y, color);
            drawPixel(x0 + y, y0 + x, color);
            drawPixel(x0 - y, y0 + x, color);
            drawPixel(x0 - x, y0 + y, color);
            drawPixel(x0 - x, y0 - y, color);
            drawPixel(x0 - y, y0 - x, color);
            drawPixel(x0 + y, y0 - x, color);
            drawPixel(x0 + x, y0 - y, color);

            y++;
            if(err < 0) {
                err += (int16_t)(2 * y + 1);
            } else {
                x--;
                err += (int16_t)(2 * (y - x) + 1);
            }
        }
    }

    void drawFastHLine(int16_t x, int16_t y, uint8_t w, uint8_t color = WHITE) {
        for(uint8_t i = 0; i < w; ++i) {
            drawPixelMode_(x + i, y, color);
        }
    }

    void drawFastVLine(int16_t x, int16_t y, uint8_t h, uint8_t color = WHITE) {
        for(uint8_t i = 0; i < h; ++i) {
            drawPixelMode_(x, y + i, color);
        }
    }

    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color = WHITE) {
        const int16_t dx = (x0 > x1) ? (x0 - x1) : (x1 - x0);
        const int16_t sx = (x0 < x1) ? 1 : -1;
        const int16_t dy = -((y0 > y1) ? (y0 - y1) : (y1 - y0));
        const int16_t sy = (y0 < y1) ? 1 : -1;
        int16_t err = dx + dy;

        while(true) {
            drawPixelMode_(x0, y0, color);
            if(x0 == x1 && y0 == y1) break;
            const int16_t e2 = (int16_t)(2 * err);
            if(e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if(e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    void drawRect(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t color = WHITE) {
        if(w == 0 || h == 0) return;
        drawFastHLine(x, y, w, color);
        drawFastHLine(x, y + h - 1, w, color);
        drawFastVLine(x, y, h, color);
        drawFastVLine(x + w - 1, y, h, color);
    }

    void fillRect(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t color = WHITE) {
        for(uint8_t i = 0; i < w; ++i) {
            drawFastVLine(x + i, y, h, color);
        }
    }

    void fillCircle(int16_t x0, int16_t y0, uint8_t r, uint8_t color = WHITE) {
        drawFastVLine(x0, y0 - r, (uint8_t)(2 * r + 1), color);

        int16_t f = 1 - r;
        int16_t ddF_x = 1;
        int16_t ddF_y = -2 * r;
        int16_t x = 0;
        int16_t y = r;

        while(x < y) {
            if(f >= 0) {
                y--;
                ddF_y += 2;
                f += ddF_y;
            }
            x++;
            ddF_x += 2;
            f += ddF_x;

            drawFastVLine(x0 + x, y0 - y, (uint8_t)(2 * y + 1), color);
            drawFastVLine(x0 - x, y0 - y, (uint8_t)(2 * y + 1), color);
            drawFastVLine(x0 + y, y0 - x, (uint8_t)(2 * x + 1), color);
            drawFastVLine(x0 - y, y0 - x, (uint8_t)(2 * x + 1), color);
        }
    }

    void drawBitmap(
        int16_t x,
        int16_t y,
        const uint8_t* bitmap,
        uint8_t w,
        uint8_t h,
        uint8_t color = WHITE) {
        if(!bitmap) return;

        const uint8_t pages = (uint8_t)((h + 7) >> 3);
        for(uint8_t page = 0; page < pages; ++page) {
            for(uint8_t i = 0; i < w; ++i) {
                const uint8_t b = pgm_read_byte(bitmap + (uint16_t)page * w + i);
                for(uint8_t bit = 0; bit < 8; ++bit) {
                    const uint8_t yy = (uint8_t)(page * 8 + bit);
                    if(yy >= h) break;

                    const int16_t px = x + i;
                    const int16_t py = y + yy;
                    const bool set = (b & (uint8_t)(1U << bit)) != 0;

                    if(color == INVERT) {
                        if(set) invertPixel_(px, py);
                    } else if(color == BLACK) {
                        drawPixel(px, py, set ? BLACK : WHITE);
                    } else {
                        drawPixel(px, py, set ? WHITE : BLACK);
                    }
                }
            }
        }
    }

    void setCursor(int16_t x, int16_t y) {
        cursor_x_ = x;
        cursor_y_ = y;
    }

    void setTextColor(uint8_t color) {
        text_color_ = color;
    }

    void setTextBackground(uint8_t bg) {
        text_bg_ = bg;
    }

    size_t write(uint8_t c) {
        if(c == '\r') return 1;
        if(c == '\n') {
            cursor_x_ = 0;
            cursor_y_ += 8;
            return 1;
        }

        if(c > 127) c = '?';
        if(c < 32) c = 32;

        const uint16_t glyph_offset = (uint16_t)c * 5U;
        for(uint8_t col = 0; col < 5; ++col) {
            const uint8_t line = pgm_read_byte(font5x7_ + glyph_offset + col);
            for(uint8_t row = 0; row < 8; ++row) {
                if(line & (uint8_t)(1U << row)) {
                    drawPixel(cursor_x_ + col, cursor_y_ + row, text_color_);
                } else {
                    drawPixel(cursor_x_ + col, cursor_y_ + row, text_bg_);
                }
            }
        }

        for(uint8_t row = 0; row < 8; ++row) {
            drawPixel(cursor_x_ + 5, cursor_y_ + row, text_bg_);
        }

        cursor_x_ += 6;
        return 1;
    }

    size_t print(const char* s) {
        if(!s) return 0;
        size_t n = 0;
        while(*s) n += write((uint8_t)*s++);
        return n;
    }

    size_t print(char c) {
        return write((uint8_t)c);
    }

    size_t print(int v) {
        return print((long)v);
    }

    size_t print(unsigned int v) {
        return print((unsigned long)v);
    }

    size_t print(long v) {
        if(v < 0) {
            size_t n = write('-');
            unsigned long uv = (unsigned long)(-(v + 1)) + 1UL;
            return n + printUnsigned_(uv);
        }
        return printUnsigned_((unsigned long)v);
    }

    size_t print(unsigned long v) {
        return printUnsigned_(v);
    }

    void drawSprite(
        int16_t x,
        int16_t y,
        const uint8_t* bmp,
        const uint8_t* mask,
        uint8_t frame,
        uint8_t mask_frame) {
        if(!bmp || !sBuffer_) return;

        const int16_t w = (int16_t)pgm_read_byte(bmp + 0);
        const int16_t h = (int16_t)pgm_read_byte(bmp + 1);
        if(!isSpriteVisible_(x, y, w, h)) return;
        const int16_t pages = (h + 7) >> 3;
        const uint16_t frame_size = (uint16_t)w * (uint16_t)pages;

        if(mask) {
            const uint8_t* sprite_data = bmp + 2 + (uint32_t)frame * (uint32_t)frame_size;
            const uint8_t* mask_data = mask + (uint32_t)mask_frame * (uint32_t)frame_size;
            blitExternalMask_(x, y, sprite_data, mask_data, w, h);
        } else {
            const uint8_t* data = bmp + 2 + (uint32_t)frame * (uint32_t)frame_size * 2u;
            blitPlusMask_(x, y, data, w, h);
        }
    }

    void setRGBled(uint8_t r, uint8_t g, uint8_t b) {
        NotificationApp* n = (NotificationApp*)furi_record_open(RECORD_NOTIFICATION);
        if(!n) return;

        notification_message(n, r ? &sequence_set_red_255 : &sequence_reset_red);
        notification_message(n, g ? &sequence_set_green_255 : &sequence_reset_green);
        notification_message(n, b ? &sequence_set_blue_255 : &sequence_reset_blue);

        furi_record_close(RECORD_NOTIFICATION);
    }

    void expectLoadDelay() {
        last_frame_ms_ = millis();
    }

    uint32_t frameCount = 0;

    static constexpr uint16_t eepromSysFlags = 1;
    // Audio mute control. 0 = audio off, non-zero = audio on
    static constexpr uint16_t eepromAudioOnOff = 2;
    // -- Addresses 3-7 are currently reserved for future use --
    // A uint16_t binary unit ID
    static constexpr uint16_t eepromUnitID = 8; // A uint16_t binary unit ID
    // An up to 6 character unit name
    // The name cannot contain 0x00, 0xFF, 0x0A, 0x0D
    // Lengths less than 6 are padded with 0x00
    static constexpr uint16_t eepromUnitName = 10;
    // -- User EEPROM space starts at address 16 --

    // --- Map of the bits in the eepromSysFlags byte --
    // Display the unit name on the logo screen
    static constexpr uint8_t sysFlagUnameBit = 0;
    static constexpr uint8_t sysFlagUnameMask = _BV(sysFlagUnameBit);
    // Show the logo sequence during boot up
    static constexpr uint8_t sysFlagShowLogoBit = 1;
    static constexpr uint8_t sysFlagShowLogoMask = _BV(sysFlagShowLogoBit);
    // Flash the RGB led during the boot logo
    static constexpr uint8_t sysFlagShowLogoLEDsBit = 2;
    static constexpr uint8_t sysFlagShowLogoLEDsMask = _BV(sysFlagShowLogoLEDsBit);

private:
    volatile bool* exit_requested_ = nullptr;

    static inline uint8_t page_mask_(int16_t p, int16_t pages, int16_t h) {
        if(p != pages - 1) return 0xFF;
        const int16_t rem = (h & 7);
        if(rem == 0) return 0xFF;
        return (uint8_t)((1u << rem) - 1u);
    }

    static inline bool isSpriteVisible_(int16_t x, int16_t y, int16_t w, int16_t h) {
        if(w <= 0 || h <= 0) return false;
        const int32_t x2 = (int32_t)x + (int32_t)w;
        const int32_t y2 = (int32_t)y + (int32_t)h;
        return (x < WIDTH) && (y < HEIGHT) && (x2 > 0) && (y2 > 0);
    }

    static inline bool clipVisibleColumns_(int16_t x, int16_t w, int16_t& start, int16_t& end) {
        if(w <= 0) return false;
        const int32_t x2 = (int32_t)x + (int32_t)w;
        if(x >= WIDTH || x2 <= 0) return false;
        start = (x < 0) ? (int16_t)(-x) : 0;
        end = (x2 > WIDTH) ? (int16_t)(WIDTH - x) : w;
        return start < end;
    }

    static inline void buildPageMasks_(uint8_t* page_masks, int16_t pages, int16_t h) {
        for(int16_t p = 0; p < pages; ++p) {
            page_masks[p] = page_mask_(p, pages, h);
        }
    }

    void blitSelfMasked_(int16_t x, int16_t y, const uint8_t* src, int16_t w, int16_t h) {
        if(!isSpriteVisible_(x, y, w, h)) return;

        int16_t col_start = 0;
        int16_t col_end = 0;
        if(!clipVisibleColumns_(x, w, col_start, col_end)) return;

        const int16_t yOffset = (int16_t)(y & 7);
        const int16_t sRow = (int16_t)(y >> 3);
        const int16_t pages = (h + 7) >> 3;
        const int16_t maxRow = (HEIGHT >> 3);
        uint8_t page_masks[32];
        buildPageMasks_(page_masks, pages, h);

        for(int16_t i = col_start; i < col_end; i++) {
            const int16_t sx = (int16_t)(x + i);
            uint8_t* dst_col = sBuffer_ + sx;

            const uint8_t* col = src + i;
            if(yOffset == 0) {
                int16_t row = sRow;
                int16_t p = 0;

                if(row < 0) {
                    p = (int16_t)(-row);
                    if(p >= pages) continue;
                    row = 0;
                    col += (int32_t)p * w;
                }
                if(row >= maxRow) continue;

                int16_t p_end = pages;
                if(row + (p_end - p) > maxRow) {
                    p_end = (int16_t)(p + (maxRow - row));
                }

                uint8_t* dst = dst_col + row * WIDTH;
                for(; p < p_end; ++p, col += w, dst += WIDTH) {
                    const uint8_t b = (uint8_t)(pgm_read_byte(col) & page_masks[p]);
                    *dst |= b;
                }
            } else {
                for(int16_t p = 0; p < pages; p++, col += w) {
                    const uint8_t b = (uint8_t)(pgm_read_byte(col) & page_masks[p]);
                    if(!b) continue;

                    const int16_t row = (int16_t)(sRow + p);
                    const uint8_t lo = (uint8_t)(b << yOffset);
                    const uint8_t hi = (uint8_t)(b >> (8 - yOffset));

                    if((uint16_t)row < (uint16_t)maxRow) {
                        dst_col[row * WIDTH] |= lo;
                    }
                    const int16_t row2 = (int16_t)(row + 1);
                    if((uint16_t)row2 < (uint16_t)maxRow) {
                        dst_col[row2 * WIDTH] |= hi;
                    }
                }
            }
        }
    }

    void blitErase_(int16_t x, int16_t y, const uint8_t* src, int16_t w, int16_t h) {
        if(!isSpriteVisible_(x, y, w, h)) return;

        int16_t col_start = 0;
        int16_t col_end = 0;
        if(!clipVisibleColumns_(x, w, col_start, col_end)) return;

        const int16_t yOffset = (int16_t)(y & 7);
        const int16_t sRow = (int16_t)(y >> 3);
        const int16_t pages = (h + 7) >> 3;
        const int16_t maxRow = (HEIGHT >> 3);
        uint8_t page_masks[32];
        buildPageMasks_(page_masks, pages, h);

        for(int16_t i = col_start; i < col_end; i++) {
            const int16_t sx = (int16_t)(x + i);
            uint8_t* dst_col = sBuffer_ + sx;

            const uint8_t* col = src + i;
            if(yOffset == 0) {
                int16_t row = sRow;
                int16_t p = 0;

                if(row < 0) {
                    p = (int16_t)(-row);
                    if(p >= pages) continue;
                    row = 0;
                    col += (int32_t)p * w;
                }
                if(row >= maxRow) continue;

                int16_t p_end = pages;
                if(row + (p_end - p) > maxRow) {
                    p_end = (int16_t)(p + (maxRow - row));
                }

                uint8_t* dst = dst_col + row * WIDTH;
                for(; p < p_end; ++p, col += w, dst += WIDTH) {
                    const uint8_t b = (uint8_t)(pgm_read_byte(col) & page_masks[p]);
                    *dst &= (uint8_t)~b;
                }
            } else {
                for(int16_t p = 0; p < pages; p++, col += w) {
                    const uint8_t b = (uint8_t)(pgm_read_byte(col) & page_masks[p]);
                    if(!b) continue;

                    const int16_t row = (int16_t)(sRow + p);
                    const uint8_t lo = (uint8_t)(b << yOffset);
                    const uint8_t hi = (uint8_t)(b >> (8 - yOffset));

                    if((uint16_t)row < (uint16_t)maxRow) {
                        dst_col[row * WIDTH] &= (uint8_t)~lo;
                    }
                    const int16_t row2 = (int16_t)(row + 1);
                    if((uint16_t)row2 < (uint16_t)maxRow) {
                        dst_col[row2 * WIDTH] &= (uint8_t)~hi;
                    }
                }
            }
        }
    }

    void blitOverwrite_(int16_t x, int16_t y, const uint8_t* src, int16_t w, int16_t h) {
        if(!isSpriteVisible_(x, y, w, h)) return;

        int16_t col_start = 0;
        int16_t col_end = 0;
        if(!clipVisibleColumns_(x, w, col_start, col_end)) return;

        const int16_t yOffset = (int16_t)(y & 7);
        const int16_t sRow = (int16_t)(y >> 3);
        const int16_t pages = (h + 7) >> 3;
        const int16_t maxRow = (HEIGHT >> 3);
        uint8_t page_masks[32];
        buildPageMasks_(page_masks, pages, h);

        for(int16_t i = col_start; i < col_end; i++) {
            const int16_t sx = (int16_t)(x + i);
            uint8_t* dst_col = sBuffer_ + sx;

            const uint8_t* col = src + i;
            if(yOffset == 0) {
                int16_t row = sRow;
                int16_t p = 0;

                if(row < 0) {
                    p = (int16_t)(-row);
                    if(p >= pages) continue;
                    row = 0;
                    col += (int32_t)p * w;
                }
                if(row >= maxRow) continue;

                int16_t p_end = pages;
                if(row + (p_end - p) > maxRow) {
                    p_end = (int16_t)(p + (maxRow - row));
                }

                uint8_t* dst = dst_col + row * WIDTH;
                for(; p < p_end; ++p, col += w, dst += WIDTH) {
                    *dst = (uint8_t)(pgm_read_byte(col) & page_masks[p]);
                }
            } else {
                for(int16_t p = 0; p < pages; p++, col += w) {
                    uint8_t b = pgm_read_byte(col);
                    const uint8_t srcMask = page_masks[p];
                    b &= srcMask;

                    const int16_t row = (int16_t)(sRow + p);

                    const uint8_t lo = (uint8_t)(b << yOffset);
                    const uint8_t hi = (uint8_t)(b >> (8 - yOffset));
                    const uint8_t maskLo = (uint8_t)(srcMask << yOffset);
                    const uint8_t maskHi = (uint8_t)(srcMask >> (8 - yOffset));

                    if((uint16_t)row < (uint16_t)maxRow) {
                        uint8_t* dst = &dst_col[row * WIDTH];
                        *dst = (uint8_t)((*dst & (uint8_t)~maskLo) | (lo & maskLo));
                    }
                    const int16_t row2 = (int16_t)(row + 1);
                    if((uint16_t)row2 < (uint16_t)maxRow) {
                        uint8_t* dst2 = &dst_col[row2 * WIDTH];
                        *dst2 = (uint8_t)((*dst2 & (uint8_t)~maskHi) | (hi & maskHi));
                    }
                }
            }
        }
    }

    void blitPlusMask_(int16_t x, int16_t y, const uint8_t* srcPairs, int16_t w, int16_t h) {
        if(!isSpriteVisible_(x, y, w, h)) return;

        int16_t col_start = 0;
        int16_t col_end = 0;
        if(!clipVisibleColumns_(x, w, col_start, col_end)) return;

        const int16_t yOffset = (int16_t)(y & 7);
        const int16_t sRow = (int16_t)(y >> 3);
        const int16_t pages = (h + 7) >> 3;
        const int16_t maxRow = (HEIGHT >> 3);
        uint8_t page_masks[32];
        buildPageMasks_(page_masks, pages, h);

        for(int16_t i = col_start; i < col_end; i++) {
            const int16_t sx = (int16_t)(x + i);
            uint8_t* dst_col = sBuffer_ + sx;
            const uint8_t* col = srcPairs + (uint16_t)i * 2u;

            if(yOffset == 0) {
                int16_t row = sRow;
                int16_t p = 0;

                if(row < 0) {
                    p = (int16_t)(-row);
                    if(p >= pages) continue;
                    row = 0;
                    col += (int32_t)p * w * 2;
                }
                if(row >= maxRow) continue;

                int16_t p_end = pages;
                if(row + (p_end - p) > maxRow) {
                    p_end = (int16_t)(p + (maxRow - row));
                }

                uint8_t* dst = dst_col + row * WIDTH;
                for(; p < p_end; ++p, col += (int32_t)w * 2, dst += WIDTH) {
                    uint8_t s = pgm_read_byte(col + 0);
                    uint8_t m = pgm_read_byte(col + 1);
                    const uint8_t srcMask = page_masks[p];
                    s &= srcMask;
                    m &= srcMask;
                    *dst = (uint8_t)((*dst & (uint8_t)~m) | (s & m));
                }
            } else {
                for(int16_t p = 0; p < pages; p++, col += (int32_t)w * 2) {
                    uint8_t s = pgm_read_byte(col + 0);
                    uint8_t m = pgm_read_byte(col + 1);
                    const uint8_t srcMask = page_masks[p];
                    s &= srcMask;
                    m &= srcMask;

                    const int16_t row = (int16_t)(sRow + p);
                    const uint8_t slo = (uint8_t)(s << yOffset);
                    const uint8_t shi = (uint8_t)(s >> (8 - yOffset));
                    const uint8_t mlo = (uint8_t)(m << yOffset);
                    const uint8_t mhi = (uint8_t)(m >> (8 - yOffset));

                    if((uint16_t)row < (uint16_t)maxRow) {
                        uint8_t* dst = &dst_col[row * WIDTH];
                        *dst = (uint8_t)((*dst & (uint8_t)~mlo) | (slo & mlo));
                    }
                    const int16_t row2 = (int16_t)(row + 1);
                    if((uint16_t)row2 < (uint16_t)maxRow) {
                        uint8_t* dst2 = &dst_col[row2 * WIDTH];
                        *dst2 = (uint8_t)((*dst2 & (uint8_t)~mhi) | (shi & mhi));
                    }
                }
            }
        }
    }

    void blitExternalMask_(
        int16_t x,
        int16_t y,
        const uint8_t* sprite,
        const uint8_t* mask,
        int16_t w,
        int16_t h) {
        if(!isSpriteVisible_(x, y, w, h)) return;

        int16_t col_start = 0;
        int16_t col_end = 0;
        if(!clipVisibleColumns_(x, w, col_start, col_end)) return;

        const int16_t yOffset = (int16_t)(y & 7);
        const int16_t sRow = (int16_t)(y >> 3);
        const int16_t pages = (h + 7) >> 3;
        const int16_t maxRow = (HEIGHT >> 3);
        uint8_t page_masks[32];
        buildPageMasks_(page_masks, pages, h);

        for(int16_t i = col_start; i < col_end; i++) {
            const int16_t sx = (int16_t)(x + i);
            uint8_t* dst_col = sBuffer_ + sx;

            const uint8_t* scol = sprite + i;
            const uint8_t* mcol = mask + i;

            if(yOffset == 0) {
                int16_t row = sRow;
                int16_t p = 0;

                if(row < 0) {
                    p = (int16_t)(-row);
                    if(p >= pages) continue;
                    row = 0;
                    scol += (int32_t)p * w;
                    mcol += (int32_t)p * w;
                }
                if(row >= maxRow) continue;

                int16_t p_end = pages;
                if(row + (p_end - p) > maxRow) {
                    p_end = (int16_t)(p + (maxRow - row));
                }

                uint8_t* dst = dst_col + row * WIDTH;
                for(; p < p_end; ++p, scol += w, mcol += w, dst += WIDTH) {
                    uint8_t s = pgm_read_byte(scol);
                    uint8_t m = pgm_read_byte(mcol);
                    const uint8_t srcMask = page_masks[p];
                    s &= srcMask;
                    m &= srcMask;
                    *dst = (uint8_t)((*dst & (uint8_t)~m) | (s & m));
                }
            } else {
                for(int16_t p = 0; p < pages; p++, scol += w, mcol += w) {
                    uint8_t s = pgm_read_byte(scol);
                    uint8_t m = pgm_read_byte(mcol);

                    const uint8_t srcMask = page_masks[p];
                    s &= srcMask;
                    m &= srcMask;

                    const int16_t row = (int16_t)(sRow + p);

                    const uint8_t slo = (uint8_t)(s << yOffset);
                    const uint8_t shi = (uint8_t)(s >> (8 - yOffset));
                    const uint8_t mlo = (uint8_t)(m << yOffset);
                    const uint8_t mhi = (uint8_t)(m >> (8 - yOffset));

                    if((uint16_t)row < (uint16_t)maxRow) {
                        uint8_t* dst = &dst_col[row * WIDTH];
                        *dst = (uint8_t)((*dst & (uint8_t)~mlo) | (slo & mlo));
                    }
                    const int16_t row2 = (int16_t)(row + 1);
                    if((uint16_t)row2 < (uint16_t)maxRow) {
                        uint8_t* dst2 = &dst_col[row2 * WIDTH];
                        *dst2 = (uint8_t)((*dst2 & (uint8_t)~mhi) | (shi & mhi));
                    }
                }
            }
        }
    }

    // static uint32_t millis() {
    //     const uint32_t tick = furi_get_tick();
    //     const uint32_t hz = furi_kernel_get_tick_frequency();
    //     if(hz == 0) return tick;
    //     return (tick * 1000u) / hz;
    // }

    size_t printUnsigned_(unsigned long v) {
        char buf[21];
        size_t len = 0;

        do {
            buf[len++] = (char)('0' + (v % 10UL));
            v /= 10UL;
        } while(v > 0 && len < sizeof(buf));

        size_t n = 0;
        while(len > 0) {
            n += write((uint8_t)buf[--len]);
        }
        return n;
    }

    void invertPixel_(int16_t x, int16_t y) {
        if(!sBuffer_) return;
        if((uint16_t)x >= WIDTH || (uint16_t)y >= HEIGHT) return;

        const uint16_t idx = (uint16_t)x + (uint16_t)((y >> 3) * WIDTH);
        sBuffer_[idx] ^= (uint8_t)(1U << (y & 7));
    }

    void drawPixelMode_(int16_t x, int16_t y, uint8_t color) {
        if(color == INVERT) {
            invertPixel_(x, y);
        } else {
            drawPixel(x, y, color);
        }
    }

    inline static const uint8_t font5x7_[] PROGMEM = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x5B, 0x4F, 0x5B, 0x3E, 0x3E, 0x6B, 0x4F, 0x6B, 0x3E, 0x1C,
            0x3E, 0x7C, 0x3E, 0x1C, 0x18, 0x3C, 0x7E, 0x3C, 0x18, 0x1C, 0x57, 0x7D, 0x57, 0x1C, 0x1C, 0x5E,
            0x7F, 0x5E, 0x1C, 0x00, 0x18, 0x3C, 0x18, 0x00, 0xFF, 0xE7, 0xC3, 0xE7, 0xFF, 0x00, 0x18, 0x24,
            0x18, 0x00, 0xFF, 0xE7, 0xDB, 0xE7, 0xFF, 0x30, 0x48, 0x3A, 0x06, 0x0E, 0x26, 0x29, 0x79, 0x29,
            0x26, 0x40, 0x7F, 0x05, 0x05, 0x07, 0x40, 0x7F, 0x05, 0x25, 0x3F, 0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
            0x7F, 0x3E, 0x1C, 0x1C, 0x08, 0x08, 0x1C, 0x1C, 0x3E, 0x7F, 0x14, 0x22, 0x7F, 0x22, 0x14, 0x5F,
            0x5F, 0x00, 0x5F, 0x5F, 0x06, 0x09, 0x7F, 0x01, 0x7F, 0x00, 0x66, 0x89, 0x95, 0x6A, 0x60, 0x60,
            0x60, 0x60, 0x60, 0x94, 0xA2, 0xFF, 0xA2, 0x94, 0x08, 0x04, 0x7E, 0x04, 0x08, 0x10, 0x20, 0x7E,
            0x20, 0x10, 0x08, 0x08, 0x2A, 0x1C, 0x08, 0x08, 0x1C, 0x2A, 0x08, 0x08, 0x1E, 0x10, 0x10, 0x10,
            0x10, 0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 0x30, 0x38, 0x3E, 0x38, 0x30, 0x06, 0x0E, 0x3E, 0x0E, 0x06,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x14,
            0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62, 0x36, 0x49,
            0x56, 0x20, 0x50, 0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x41, 0x22,
            0x1C, 0x00, 0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x80, 0x70, 0x30,
            0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x60, 0x60, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02,
            0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x72, 0x49, 0x49, 0x49, 0x46, 0x21,
            0x41, 0x49, 0x4D, 0x33, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A,
            0x49, 0x49, 0x31, 0x41, 0x21, 0x11, 0x09, 0x07, 0x36, 0x49, 0x49, 0x49, 0x36, 0x46, 0x49, 0x49,
            0x29, 0x1E, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x40, 0x34, 0x00, 0x00, 0x00, 0x08, 0x14, 0x22,
            0x41, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x59, 0x09, 0x06,
            0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E,
            0x41, 0x41, 0x41, 0x22, 0x7F, 0x41, 0x41, 0x41, 0x3E, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09,
            0x09, 0x09, 0x01, 0x3E, 0x41, 0x41, 0x51, 0x73, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F,
            0x41, 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40,
            0x40, 0x7F, 0x02, 0x1C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
            0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x26,
            0x49, 0x49, 0x49, 0x32, 0x03, 0x01, 0x7F, 0x01, 0x03, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x1F, 0x20,
            0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63, 0x03, 0x04, 0x78,
            0x04, 0x03, 0x61, 0x59, 0x49, 0x4D, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x41, 0x02, 0x04, 0x08, 0x10,
            0x20, 0x00, 0x41, 0x41, 0x41, 0x7F, 0x04, 0x02, 0x01, 0x02, 0x04, 0x40, 0x40, 0x40, 0x40, 0x40,
            0x00, 0x03, 0x07, 0x08, 0x00, 0x20, 0x54, 0x54, 0x78, 0x40, 0x7F, 0x28, 0x44, 0x44, 0x38, 0x38,
            0x44, 0x44, 0x44, 0x28, 0x38, 0x44, 0x44, 0x28, 0x7F, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x08,
            0x7E, 0x09, 0x02, 0x18, 0xA4, 0xA4, 0x9C, 0x78, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D,
            0x40, 0x00, 0x20, 0x40, 0x40, 0x3D, 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40,
            0x00, 0x7C, 0x04, 0x78, 0x04, 0x78, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38,
            0xFC, 0x18, 0x24, 0x24, 0x18, 0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48,
            0x54, 0x54, 0x54, 0x24, 0x04, 0x04, 0x3F, 0x44, 0x24, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20,
            0x40, 0x20, 0x1C, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28, 0x44, 0x4C, 0x90, 0x90,
            0x90, 0x7C, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x77, 0x00,
            0x00, 0x00, 0x41, 0x36, 0x08, 0x00, 0x02, 0x01, 0x02, 0x04, 0x02, 0x3C, 0x26, 0x23, 0x26, 0x3C,
    };

    static uint8_t mapInputToArduboyMask_(uint8_t in) {
#if (INPUT_UP == UP_BUTTON) && (INPUT_DOWN == DOWN_BUTTON) && (INPUT_LEFT == LEFT_BUTTON) && \
    (INPUT_RIGHT == RIGHT_BUTTON) && (INPUT_A == A_BUTTON) && (INPUT_B == B_BUTTON)
        return in;
#else
        uint8_t out = 0;
        if(in & INPUT_UP) out |= UP_BUTTON;
        if(in & INPUT_DOWN) out |= DOWN_BUTTON;
        if(in & INPUT_LEFT) out |= LEFT_BUTTON;
        if(in & INPUT_RIGHT) out |= RIGHT_BUTTON;
        if(in & INPUT_B) out |= B_BUTTON;
        if(in & INPUT_A) out |= A_BUTTON;
        return out;
#endif
    }

    FuriMutex* game_mutex_ = nullptr;
    bool external_timing_ = false;

    uint8_t* sBuffer_ = nullptr;
    volatile uint8_t* input_state_ = nullptr;
    InputContext input_ctx_{};

    uint32_t frame_duration_ms_ = 16;
    uint32_t last_frame_ms_ = 0;

    uint8_t cur_buttons_ = 0;
    uint8_t prev_buttons_ = 0;
    int16_t cursor_x_ = 0;
    int16_t cursor_y_ = 0;
    uint8_t text_color_ = WHITE;
    uint8_t text_bg_ = BLACK;
};

class Arduboy2 : public Arduboy2Base {};

class BeepPin1 {
public:
    void begin() {
        // Audio init/state is controlled by ArduboyAudio in main runtime.
    }

    uint16_t freq(uint16_t frequency) {
        return frequency;
    }

    void tone(uint16_t frequency, uint16_t duration_frames) {
        if(duration_frames == 0) {
            tones_.noTone();
            return;
        }

        // Arduboy BeepPin1 duration is frame-based (called from timer() each frame).
        uint32_t duration_ms = ((uint32_t)duration_frames * 1000u + 30u) / 60u;
        if(duration_ms == 0) duration_ms = 1;
        tones_.tone(frequency, (uint16_t)duration_ms);
    }

    void timer() {
    }

private:
    ArduboyTones tones_;
};

class Sprites {
public:
    static void setArduboy(Arduboy2Base* a) {
        ab_ = a;
    }

    static void drawOverwrite(int16_t x, int16_t y, const uint8_t* bmp, uint8_t frame = 0) {
        if(!ab_ || !bmp) return;
        ab_->drawSolidBitmapFrame(x, y, bmp, frame);
    }

    static void drawSelfMasked(int16_t x, int16_t y, const uint8_t* bmp, uint8_t frame = 0) {
        if(!ab_ || !bmp) return;
        ab_->drawBitmapFrame(x, y, bmp, frame);
    }

    static void drawErase(int16_t x, int16_t y, const uint8_t* bmp, uint8_t frame = 0) {
        if(!ab_ || !bmp) return;
        ab_->eraseBitmapFrame(x, y, bmp, frame);
    }

    static void drawPlusMask(int16_t x, int16_t y, const uint8_t* plusmask, uint8_t frame = 0) {
        if(!ab_) return;
        ab_->drawPlusMask(x, y, plusmask, frame);
    }

private:
    inline static Arduboy2Base* ab_ = nullptr;
};
