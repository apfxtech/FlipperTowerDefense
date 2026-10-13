/**
  @file   main.cpp
  @author apfxtech
  @brief  application shell: viewport, input, timing, sound and saves

  MicroTD - Tower Defense Game, Flipper Zero port

  apfxtech, 2026, license: MIT (see LICENSE)

  SPDX-License-Identifier: MIT
*/

#include <furi.h>
#include <furi_hal_rtc.h>
#include <furi_hal_speaker.h>
#include <gui/gui.h>
#include <input/input.h>
#include <storage/storage.h>

#include <string.h>

#include "microtd.h"
#include "render.h"

#define MICROTD_FRAME_RATE  60
#define MICROTD_TONE_VOLUME 1.0f
#define MICROTD_SAVE_PATH   APP_DATA_PATH("eeprom.bin")

#define BACK_EXIT_HOLD_MS 800

typedef struct {
    FuriMutex* mutex;
    FuriMessageQueue* input_queue;
    ViewPort* view_port;
    Gui* gui;

    uint8_t held;
    uint8_t press_latch;
    bool back_down;
    bool back_as_b;
    uint32_t back_tick;
    bool back_long;
    bool exit_requested;

    bool speaker_owned;
    bool tone_active;
    uint32_t tone_until;

    uint8_t records[MICROTD_MAPS];
} MicroTdApp;

static MicroTdApp* app_instance = NULL;

void platform_tone(uint16_t frequency, uint8_t frames) {
    MicroTdApp* app = app_instance;
    if(!app || !app->speaker_owned || frequency == 0) return;

    furi_hal_speaker_start((float)frequency, MICROTD_TONE_VOLUME);
    app->tone_active = true;
    app->tone_until =
        furi_get_tick() + (furi_kernel_get_tick_frequency() * frames) / MICROTD_FRAME_RATE;
}

static void tone_expire(MicroTdApp* app) {
    if(!app->tone_active) return;
    if((int32_t)(furi_get_tick() - app->tone_until) < 0) return;

    furi_hal_speaker_stop();
    app->tone_active = false;
}

static void records_load(uint8_t records[MICROTD_MAPS]) {
    Storage* storage = (Storage*)furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);

    if(storage_file_open(file, MICROTD_SAVE_PATH, FSAM_READ, FSOM_OPEN_EXISTING)) {
        storage_file_read(file, records, MICROTD_MAPS);
        storage_file_close(file);
    }

    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

static void records_store(const uint8_t records[MICROTD_MAPS]) {
    Storage* storage = (Storage*)furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);

    if(storage_file_open(file, MICROTD_SAVE_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        storage_file_write(file, records, MICROTD_MAPS);
        storage_file_close(file);
    }

    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

static uint8_t button_from_key(InputKey key) {
    switch(key) {
    case InputKeyUp:
        return MICROTD_UP;
    case InputKeyDown:
        return MICROTD_DOWN;
    case InputKeyLeft:
        return MICROTD_LEFT;
    case InputKeyRight:
        return MICROTD_RIGHT;
    case InputKeyOk:
        return MICROTD_A;
    case InputKeyBack:
        return MICROTD_B;
    default:
        return 0;
    }
}

static void back_becomes_b(MicroTdApp* app) {
    if(app->back_as_b) return;
    app->back_as_b = true;
    app->held |= MICROTD_B;
    app->press_latch |= MICROTD_B;
}

static void input_apply(MicroTdApp* app, const InputEvent* event) {
    if(event->key == InputKeyBack) {
        if(event->type == InputTypePress) {
            app->back_down = true;
            app->back_as_b = false;
            app->back_tick = furi_get_tick();
            if(app->held != 0) back_becomes_b(app);
        } else if(event->type == InputTypeRelease) {
            if(app->back_down && !app->back_as_b) app->press_latch |= MICROTD_B;
            app->held &= (uint8_t)~MICROTD_B;
            app->back_down = false;
            app->back_as_b = false;
        }
        return;
    }

    const uint8_t bit = button_from_key(event->key);
    if(!bit) return;

    if(event->type == InputTypePress) {
        app->held |= bit;
        app->press_latch |= bit;
        if(app->back_down) back_becomes_b(app);
    } else if(event->type == InputTypeRepeat) {
        app->held |= bit;
    } else if(event->type == InputTypeRelease) {
        app->held &= (uint8_t)~bit;
    }
}

static void back_gesture_check(MicroTdApp* app) {
    if(!app->back_down || app->back_as_b) return;

    const uint32_t hold = furi_get_tick() - app->back_tick;
    if(hold < furi_kernel_get_tick_frequency() * BACK_EXIT_HOLD_MS / 1000) return;

    app->back_long = true;
    app->back_down = false;
}

static void frame_advance(MicroTdApp* app) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    if(app->back_long) {
        app->back_long = false;
        app->held = 0;
        app->press_latch = 0;

        if(microtd_back_long()) {
            app->exit_requested = true;
        }
    }

    if(!app->exit_requested) {
        const uint8_t press = app->press_latch;
        app->press_latch = 0;
        microtd_update(app->held, press);
    }

    furi_mutex_release(app->mutex);

    if(microtd_take_records(app->records)) {
        records_store(app->records);
    }
}

static void draw_callback(Canvas* canvas, void* context) {
    MicroTdApp* app = (MicroTdApp*)context;

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    gfx_bind(canvas);
    microtd_draw();
    gfx_unbind();
    furi_mutex_release(app->mutex);
}

static void input_callback(InputEvent* event, void* context) {
    MicroTdApp* app = (MicroTdApp*)context;
    furi_message_queue_put(app->input_queue, event, 0);
}

extern "C" int32_t microtd_app(void* p) {
    UNUSED(p);

    MicroTdApp* app = (MicroTdApp*)malloc(sizeof(MicroTdApp));
    memset(app, 0, sizeof(MicroTdApp));
    app_instance = app;

    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->input_queue = furi_message_queue_alloc(16, sizeof(InputEvent));

    app->speaker_owned = furi_hal_speaker_acquire(100);

    records_load(app->records);
    microtd_start(!furi_hal_rtc_is_flag_set(FuriHalRtcFlagStealthMode), app->records);

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app);

    app->gui = (Gui*)furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    const uint32_t period = furi_kernel_get_tick_frequency() / MICROTD_FRAME_RATE;
    uint32_t next_frame = furi_get_tick() + period;

    while(!app->exit_requested) {
        const uint32_t now = furi_get_tick();
        const int32_t remaining = (int32_t)(next_frame - now);

        InputEvent event;
        if(remaining > 0 &&
           furi_message_queue_get(app->input_queue, &event, (uint32_t)remaining) == FuriStatusOk) {
            input_apply(app, &event);
            back_gesture_check(app);
            continue;
        }

        back_gesture_check(app);

        next_frame += period;
        if((int32_t)(furi_get_tick() - next_frame) > (int32_t)period) {
            next_frame = furi_get_tick() + period;
        }

        tone_expire(app);
        frame_advance(app);
        view_port_update(app->view_port);
    }

    gui_remove_view_port(app->gui, app->view_port);
    furi_record_close(RECORD_GUI);
    view_port_free(app->view_port);

    if(app->speaker_owned) {
        if(app->tone_active) furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }

    if(microtd_take_records(app->records)) {
        records_store(app->records);
    }

    furi_message_queue_free(app->input_queue);
    furi_mutex_free(app->mutex);

    app_instance = NULL;
    free(app);

    return 0;
}
