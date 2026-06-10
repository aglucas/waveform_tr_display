#pragma once
#include "lvgl.h"
#include "i2c_rx.h"

/* Build all gauge widgets on `parent` (pass lv_screen_active()). */
void gauge_ui_init(lv_obj_t *parent);

/* Update needle and all data labels from fresh telemetry. */
void gauge_ui_update_telemetry(const telemetry_t *t);

/* Update odometer readouts. */
void gauge_ui_update_mileage(double total_miles, double trip_miles);

/* Toggle the blink state of any active alert indicators. */
void gauge_ui_blink_tick(void);

/* Register a callback invoked (from LVGL task context) when the ODO/trip
 * area is tapped. Use to trigger a trip reset via a flag — do not call
 * LVGL functions directly inside the callback. */
void gauge_ui_set_trip_cb(void (*cb)(void));

/* Register a callback invoked when the temp warning circle is tapped
 * or transitions to red. Set a flag in the callback — do not call
 * LVGL or blocking functions directly. */
void gauge_ui_set_sound_cb(void (*cb)(void));
void gauge_ui_set_alert_sound_cb(void (*cb)(void));
