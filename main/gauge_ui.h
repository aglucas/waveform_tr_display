#pragma once
#include "lvgl.h"
#include "rs485_rx.h"

/* Build all gauge widgets on `parent` (pass lv_screen_active()). */
void gauge_ui_init(lv_obj_t *parent);

/* Update needle and all data labels from fresh telemetry. */
void gauge_ui_update_telemetry(const telemetry_t *t);

/* Update odometer readouts. */
void gauge_ui_update_mileage(double total_miles, double trip_miles);

/* Update the efficiency readout (Wh/mile, shown under the SOC block). Pass the
 * energy (Wh) and distance (miles) for each of the three meters; the readout
 * shows whichever is selected, cycled by tapping the large readout:
 *   trip   'T' — since the last trip reset
 *   charge 'C' — since the last transition into Charge
 *   roll   'M' — over the last 10 miles of trip distance (window deltas) */
void gauge_ui_update_energy(double trip_wh, double trip_miles,
                            double charge_wh, double charge_miles,
                            double roll_wh, double roll_miles);

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

/* Register a callback invoked when the needle pivot hub is tapped.
 * Use to trigger a brightness step via a flag — do not call LVGL or
 * blocking functions directly inside the callback. */
void gauge_ui_set_brightness_cb(void (*cb)(void));

/* Set the needle-pivot hub ring color: true = green (a PLP/power-fail save
 * has landed on disk — either just now, or confirmed at boot), false = red
 * (a normal/periodic save, or nothing confirmed since going into Run). Must
 * be called from LVGL task context (i.e. with the display already locked). */
void gauge_ui_set_hub_ok(bool ok);
