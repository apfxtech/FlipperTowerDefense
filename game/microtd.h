/**
  @file   microtd.h
  @author apfxtech
  @brief  interface between the application shell and the game core

  MicroTD - Tower Defense Game, Flipper Zero port

  apfxtech, 2026, license: MIT (see LICENSE)

  SPDX-License-Identifier: MIT
*/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MICROTD_MAPS 5

#define MICROTD_UP    0x01
#define MICROTD_DOWN  0x02
#define MICROTD_LEFT  0x04
#define MICROTD_RIGHT 0x08
#define MICROTD_A     0x10
#define MICROTD_B     0x20

#ifdef __cplusplus
extern "C" {
#endif

void microtd_start(bool sound, const uint8_t records[MICROTD_MAPS]);
void microtd_update(uint8_t held, uint8_t press_edges);
void microtd_draw(void);
bool microtd_back_long(void);
bool microtd_take_records(uint8_t records[MICROTD_MAPS]);

void platform_tone(uint16_t frequency, uint8_t frames);

#ifdef __cplusplus
}
#endif
