#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>

#include <stdlib.h>
#include <string.h>

#include "lib/Arduboy2.h"
#include "lib/Arduino.h"
#include "lib/EEPROM.h"

#define TARGET_FRAMERATE 60

#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64
#define BUFFER_SIZE    (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8)

typedef struct {
    uint8_t screen_buffer[BUFFER_SIZE];
    uint8_t front_buffer[BUFFER_SIZE];

    Gui* gui;
    Canvas* canvas;
    FuriTimer* timer;
    FuriMutex* fb_mutex;
    FuriMutex* game_mutex;
    FuriPubSub* input_events;
    FuriPubSubSubscription* input_sub;

    volatile uint8_t input_state;
    volatile bool exit_requested;
    volatile bool in_frame;
} FlipperState;

extern void setup();
extern void loop();
extern Arduboy2 arduboy;

static FlipperState* g_state = NULL;

static void framebuffer_commit_callback(
    uint8_t* data,
    size_t size,
    CanvasOrientation orientation,
    void* context) {
    FlipperState* state = (FlipperState*)context;
    if(!state || !data || size < BUFFER_SIZE) return;
    (void)orientation;

    if(furi_mutex_acquire(state->fb_mutex, 0) != FuriStatusOk) return;
    for(size_t i = 0; i < BUFFER_SIZE; i++) {
        data[i] = (uint8_t)(state->front_buffer[i] ^ 0xFF);
    }
    furi_mutex_release(state->fb_mutex);
}

static void input_events_callback(const void* value, void* ctx) {
    if(!value || !ctx) return;

    FlipperState* state = (FlipperState*)ctx;
    const InputEvent* event = (const InputEvent*)value;

    if(event->key == InputKeyBack && event->type == InputTypeLong) {
        state->exit_requested = true;
        return;
    }

    InputEvent copy = *event;
    Arduboy2Base::FlipperInputCallback(&copy, arduboy.inputContext());
}

static void timer_callback(void* ctx) {
    FlipperState* state = (FlipperState*)ctx;
    if(!state || state->in_frame) return;
    state->in_frame = true;

    if(furi_mutex_acquire(state->game_mutex, 0) != FuriStatusOk) {
        state->in_frame = false;
        return;
    }

    loop();

    furi_mutex_acquire(state->fb_mutex, FuriWaitForever);
    memcpy(state->front_buffer, state->screen_buffer, BUFFER_SIZE);
    furi_mutex_release(state->fb_mutex);

    if(state->canvas) canvas_commit(state->canvas);

    furi_mutex_release(state->game_mutex);
    state->in_frame = false;
}

extern "C" int32_t arduboy_app(void* p) {
    UNUSED(p);

    bool fb_callback_added = false;
    int32_t rc = -1;

    g_state = (FlipperState*)malloc(sizeof(FlipperState));
    if(!g_state) return -1;
    memset(g_state, 0, sizeof(FlipperState));

    do {
        g_state->fb_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
        g_state->game_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
        if(!g_state->fb_mutex || !g_state->game_mutex) break;

        memset(g_state->screen_buffer, 0x00, BUFFER_SIZE);
        memset(g_state->front_buffer, 0x00, BUFFER_SIZE);
        g_state->input_state = 0;
        g_state->exit_requested = false;
        g_state->in_frame = false;

        arduboy.begin(
            g_state->screen_buffer,
            &g_state->input_state,
            g_state->game_mutex,
            &g_state->exit_requested);
        Sprites::setArduboy(&arduboy);

        g_state->gui = (Gui*)furi_record_open(RECORD_GUI);
        if(!g_state->gui) break;

        gui_add_framebuffer_callback(g_state->gui, framebuffer_commit_callback, g_state);
        fb_callback_added = true;

        g_state->canvas = gui_direct_draw_acquire(g_state->gui);
        if(!g_state->canvas) break;

        g_state->input_events = (FuriPubSub*)furi_record_open(RECORD_INPUT_EVENTS);
        if(!g_state->input_events) break;

        g_state->input_sub =
            furi_pubsub_subscribe(g_state->input_events, input_events_callback, g_state);
        if(!g_state->input_sub) break;

        furi_mutex_acquire(g_state->game_mutex, FuriWaitForever);
        arduboy.audio.begin();
        EEPROM.begin();
        setup();
        furi_mutex_acquire(g_state->fb_mutex, FuriWaitForever);
        memcpy(g_state->front_buffer, g_state->screen_buffer, BUFFER_SIZE);
        furi_mutex_release(g_state->fb_mutex);
        if(g_state->canvas) canvas_commit(g_state->canvas);
        furi_mutex_release(g_state->game_mutex);

        g_state->timer = furi_timer_alloc(timer_callback, FuriTimerTypePeriodic, g_state);
        if(!g_state->timer) break;

        const uint32_t tick_hz = furi_kernel_get_tick_frequency();
        uint32_t period = (tick_hz + (TARGET_FRAMERATE / 2)) / TARGET_FRAMERATE;
        if(period == 0) period = 1;
        furi_timer_start(g_state->timer, period);

        while(!g_state->exit_requested) {
            furi_delay_ms(50);
        }

        rc = 0;
    } while(false);

    EEPROM.commit();
    arduboy_tone_sound_system_deinit();

    if(g_state->timer) {
        furi_timer_stop(g_state->timer);
        furi_timer_free(g_state->timer);
        g_state->timer = NULL;
    }

    if(g_state->input_sub && g_state->input_events) {
        furi_pubsub_unsubscribe(g_state->input_events, g_state->input_sub);
        g_state->input_sub = NULL;
    }

    if(g_state->input_events) {
        furi_record_close(RECORD_INPUT_EVENTS);
        g_state->input_events = NULL;
    }

    if(g_state->gui) {
        if(g_state->canvas) {
            gui_direct_draw_release(g_state->gui);
            g_state->canvas = NULL;
        }
        if(fb_callback_added) {
            gui_remove_framebuffer_callback(g_state->gui, framebuffer_commit_callback, g_state);
        }
        furi_record_close(RECORD_GUI);
        g_state->gui = NULL;
    }

    if(g_state->fb_mutex) {
        furi_mutex_free(g_state->fb_mutex);
        g_state->fb_mutex = NULL;
    }
    if(g_state->game_mutex) {
        furi_mutex_free(g_state->game_mutex);
        g_state->game_mutex = NULL;
    }

    free(g_state);
    g_state = NULL;

    return rc;
}
