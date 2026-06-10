#include <stdio.h>
#include <string.h>
#include "gauge_ui.h"
#include "triumph_logo.h"

LV_FONT_DECLARE(lv_font_montserrat_bold_28)
LV_FONT_DECLARE(lv_font_montserrat_bold_36)
LV_FONT_DECLARE(lv_font_montserrat_bold_44)

/* ── Display geometry (800 × 800) ─────────────────────────────────────────── */
#define CX  400
#define CY  400

/* Gauge sweep: 0 mph at 7 o'clock (210° CW from 12), 300° total → 5 o'clock.
 * lv_scale rotation: 0° = 3 o'clock, CW.  7 o'clock = 120° from 3 o'clock. */
#define MAX_SPEED        100
#define NUM_STEPS        10
#define SWEEP_TOTAL_DEG  300
#define SCALE_ROTATION   120

#define SCALE_DIAM       760   /* nearly full display                   */
#define NEEDLE_LEN       320   /* centre → tip, px                      */
#define TICK_MINOR_LEN   13
#define TICK_MAJOR_LEN   22

/* Warning indicator positions (scaled ~1.67× from 480-px CircuitPython) */
#define SOC_X            200
#define TEMP_X           600
#define WARN_Y           CY
#define WARN_OUTER_R     30
#define WARN_INNER_R     22
#define WARN_ROW_H       50    /* vertical gap between label rows       */

#define ODO_Y            680

/* Colours */
#define COL_RED     lv_color_hex(0xFF0000)
#define COL_GREEN   lv_color_hex(0x00AA00)
#define COL_YELLOW  lv_color_hex(0xFFFF00)
#define COL_WHITE   lv_color_hex(0xFFFFFF)
#define COL_BLACK   lv_color_hex(0x000000)
#define COL_BG      lv_color_hex(0x111111)

/* Warning thresholds — same values as CircuitPython */
#define SOC_WARN_PCT   15
#define SOC_ALERT_PCT  10

typedef struct { float la, lw, hw, ha; } thresh_t;

static const thresh_t TH_BAT = { -10,   0,  40,  50 };
static const thresh_t TH_MOT = { -15,  -5,  80, 100 };
static const thresh_t TH_INV = { -15,  -5,  80, 100 };
static const thresh_t TH_ISA = { -30, -20,  75, 100 };
static const thresh_t TH_UDC = { 270, 300, 400, 410 };

/* Widget handles */
static lv_obj_t *g_scale;
static lv_obj_t *g_needle;
static lv_obj_t *g_soc_inner;
static lv_obj_t *g_temp_inner;
static lv_obj_t *g_temp_outer;
static lv_obj_t *g_bat_temp_lbl;
static lv_obj_t *g_soc_pct_lbl;
static lv_obj_t *g_udc_lbl;
static lv_obj_t *g_temp_val_lbl;
static lv_obj_t *g_temp_src_lbl;
static lv_obj_t *g_odo_lbl;
static lv_obj_t *g_odo_bg;

/* Trip reset callback */
static void (*s_trip_cb)(void) = NULL;

static void odo_click_cb(lv_event_t *e)
{
    if (s_trip_cb) s_trip_cb();
}

/* Sound callbacks — tap uses normal volume, alert uses full volume */
static void (*s_sound_cb)(void)  = NULL;   /* temp circle tapped */
static void (*s_alert_cb)(void)  = NULL;   /* temp transitions to red */
static bool s_prev_temp_alert = false;

static void temp_click_cb(lv_event_t *e)
{
    if (s_sound_cb) s_sound_cb();
}

/* Alert blink state */
static bool g_soc_alert  = false;
static bool g_temp_alert = false;
static bool g_udc_alert  = false;
static bool g_blink_on   = true;

/* Persistent styles */
static lv_style_t s_scale_bg;
static lv_style_t s_ticks_minor;
static lv_style_t s_ticks_major;
static lv_style_t s_needle_line;

/* ─── Helpers ────────────────────────────────────────────────────────────── */

static int severity(float v, const thresh_t *t)
{
    if (v <= t->la || v >= t->ha) return 2;
    if (v <= t->lw || v >= t->hw) return 1;
    return 0;
}

/* Create a filled circle widget positioned so its centre is at (cx, cy). */
static lv_obj_t *circle(lv_obj_t *parent, int cx, int cy, int r, lv_color_t col)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_set_size(o, r * 2, r * 2);
    lv_obj_set_pos(o, cx - r, cy - r);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(o, col, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

/* Create a label, centred at (cx, cy), transparent background. */
static lv_obj_t *centred_label(lv_obj_t *parent, int cx, int cy,
                                const char *txt, lv_color_t col,
                                const lv_font_t *font)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_TRANSP, 0);
    lv_obj_set_width(l, LV_SIZE_CONTENT);
    lv_obj_set_height(l, LV_SIZE_CONTENT);
    /* Align after LVGL has computed the size on the next layout pass.
     * We use a fixed-position approach: the label's left-top is offset
     * so its computed content fits centred on (cx, cy).
     * lv_obj_update_layout forces an immediate measure. */
    lv_obj_update_layout(l);
    lv_obj_set_pos(l, cx - lv_obj_get_width(l) / 2,
                      cy - lv_obj_get_height(l) / 2);
    return l;
}

/* Create a label with a fixed pixel width and centred text alignment.
 * Position is derived from the font's known line_height so lv_obj_update_layout
 * is never needed at update time — lv_label_set_text() alone is sufficient. */
static lv_obj_t *fixed_label(lv_obj_t *parent, int cx, int cy, int w, int lines,
                               const char *txt, lv_color_t col, const lv_font_t *font)
{
    int h = font->line_height * lines + 4 * (lines - 1);
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(l, w);
    lv_obj_set_height(l, LV_SIZE_CONTENT);
    lv_obj_set_pos(l, cx - w / 2, cy - h / 2);
    return l;
}

/* ─── gauge_ui_init ──────────────────────────────────────────────────────── */

void gauge_ui_init(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, COL_BG, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    /* Background logo – drawn first so everything else layers on top */
    lv_obj_t *bg_img = lv_image_create(parent);
    lv_image_set_src(bg_img, &triumph_logo);
    lv_obj_center(bg_img);

    /* Scale background */
    lv_style_init(&s_scale_bg);
    lv_style_set_bg_opa(&s_scale_bg, LV_OPA_TRANSP);
    lv_style_set_border_width(&s_scale_bg, 0);
    lv_style_set_pad_all(&s_scale_bg, 0);

    /* Minor tick style (LV_PART_ITEMS) */
    lv_style_init(&s_ticks_minor);
    lv_style_set_line_color(&s_ticks_minor, COL_RED);
    lv_style_set_line_width(&s_ticks_minor, 2);
    lv_style_set_length(&s_ticks_minor, TICK_MINOR_LEN);

    /* Major tick + label style (LV_PART_INDICATOR) */
    lv_style_init(&s_ticks_major);
    lv_style_set_line_color(&s_ticks_major, COL_RED);
    lv_style_set_line_width(&s_ticks_major, 3);
    lv_style_set_length(&s_ticks_major, TICK_MAJOR_LEN);
    lv_style_set_text_color(&s_ticks_major, COL_RED);
    lv_style_set_text_font(&s_ticks_major, &lv_font_montserrat_bold_44);

    /* Scale widget */
    g_scale = lv_scale_create(parent);
    lv_obj_set_size(g_scale, SCALE_DIAM, SCALE_DIAM);
    lv_obj_center(g_scale);
    lv_scale_set_mode(g_scale, LV_SCALE_MODE_ROUND_INNER);
    lv_scale_set_range(g_scale, 0, MAX_SPEED);
    lv_scale_set_total_tick_count(g_scale, MAX_SPEED + 1);
    lv_scale_set_major_tick_every(g_scale, NUM_STEPS);
    lv_scale_set_angle_range(g_scale, SWEEP_TOTAL_DEG);
    lv_scale_set_rotation(g_scale, SCALE_ROTATION);
    lv_obj_add_style(g_scale, &s_scale_bg,    0);
    lv_obj_add_style(g_scale, &s_ticks_minor, LV_PART_ITEMS);
    lv_obj_add_style(g_scale, &s_ticks_major, LV_PART_INDICATOR);

    /* Needle line (child of scale) */
    lv_style_init(&s_needle_line);
    lv_style_set_line_color(&s_needle_line, COL_RED);
    lv_style_set_line_width(&s_needle_line, 5);
    lv_style_set_line_rounded(&s_needle_line, true);

    g_needle = lv_line_create(g_scale);
    lv_obj_add_style(g_needle, &s_needle_line, 0);
    lv_scale_set_line_needle_value(g_scale, g_needle, NEEDLE_LEN, 0);

    /* Hub dot (drawn on top of needle base) */
    lv_obj_t *hub = circle(parent, CX, CY, 12, COL_RED);
    (void)hub;

    /* SOC warning indicator */
    circle(parent, SOC_X, WARN_Y, WARN_OUTER_R, COL_WHITE);
    g_soc_inner = circle(parent, SOC_X, WARN_Y, WARN_INNER_R, COL_GREEN);
    centred_label(parent, SOC_X, WARN_Y + WARN_ROW_H, "SOC",
                  COL_RED, &lv_font_montserrat_bold_36);

    /* TEMP warning indicator */
    g_temp_outer = circle(parent, TEMP_X, WARN_Y, WARN_OUTER_R, COL_WHITE);
    lv_obj_add_flag(g_temp_outer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_temp_outer, temp_click_cb, LV_EVENT_CLICKED, NULL);
    g_temp_inner = circle(parent, TEMP_X, WARN_Y, WARN_INNER_R, COL_GREEN);
    centred_label(parent, TEMP_X, WARN_Y + WARN_ROW_H, "TMP",
                  COL_RED, &lv_font_montserrat_bold_36);

    const lv_font_t *df = &lv_font_montserrat_bold_44;

    /* Left column: bat temp, SOC %, DC bus V (above SOC circle, top→bottom)
     * Fixed-width labels — text updates need only lv_label_set_text(), no recentre. */
    g_bat_temp_lbl = fixed_label(parent, SOC_X, WARN_Y - WARN_ROW_H * 3, 150, 1,
                                 "20C",  COL_RED, df);
    g_soc_pct_lbl  = fixed_label(parent, SOC_X, WARN_Y - WARN_ROW_H * 2, 150, 1,
                                 "100%", COL_RED, df);
    g_udc_lbl      = fixed_label(parent, SOC_X, WARN_Y - WARN_ROW_H,     150, 1,
                                 "---",  COL_RED, df);

    /* Right column: worst-temp value and source (hidden until warning) */
    g_temp_val_lbl = fixed_label(parent, TEMP_X, WARN_Y - WARN_ROW_H * 2, 150, 1,
                                 "20C", COL_RED, df);
    lv_obj_add_flag(g_temp_val_lbl, LV_OBJ_FLAG_HIDDEN);

    g_temp_src_lbl = fixed_label(parent, TEMP_X, WARN_Y - WARN_ROW_H,     150, 1,
                                 "BAT", COL_RED, df);
    lv_obj_add_flag(g_temp_src_lbl, LV_OBJ_FLAG_HIDDEN);

    /* Odometer */
    g_odo_bg = lv_obj_create(parent);
    lv_obj_set_size(g_odo_bg, 240, 120);
    lv_obj_set_pos(g_odo_bg, CX - 120, ODO_Y - 60);
    lv_obj_set_style_bg_color(g_odo_bg, COL_BLACK, 0);
    lv_obj_set_style_bg_opa(g_odo_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_odo_bg, 0, 0);
    lv_obj_set_style_radius(g_odo_bg, 6, 0);
    lv_obj_clear_flag(g_odo_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_odo_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_odo_bg, odo_click_cb, LV_EVENT_CLICKED, NULL);

    g_odo_lbl = fixed_label(parent, CX, ODO_Y, 280, 2,
                            "ODO: 0\nTRP: 0.0", COL_RED, &lv_font_montserrat_bold_36);
}

/* ─── gauge_ui_update_telemetry ─────────────────────────────────────────── */

void gauge_ui_update_telemetry(const telemetry_t *t)
{
    char buf[32];

    /* Needle */
    int spd = (int)(t->speed_mph < 0 ? 0 : t->speed_mph > MAX_SPEED
                    ? MAX_SPEED : t->speed_mph);
    lv_scale_set_line_needle_value(g_scale, g_needle, NEEDLE_LEN, spd);

    /* DC bus voltage label */
    snprintf(buf, sizeof(buf), "%dV", (int)t->udc_v);
    lv_label_set_text(g_udc_lbl, buf);

    /* SOC % label */
    snprintf(buf, sizeof(buf), "%d%%", (int)t->soc_pct);
    lv_label_set_text(g_soc_pct_lbl, buf);

    /* Battery temperature label */
    snprintf(buf, sizeof(buf), "%dC", (int)t->temp_bat_c);
    lv_label_set_text(g_bat_temp_lbl, buf);

    /* SOC indicator: worst of SOC% and bus voltage */
    int soc_sev = (t->soc_pct < SOC_ALERT_PCT) ? 2
                : (t->soc_pct < SOC_WARN_PCT)  ? 1 : 0;
    int udc_sev = severity(t->udc_v, &TH_UDC);
    int combined = soc_sev > udc_sev ? soc_sev : udc_sev;

    g_soc_alert = false;
    if (combined == 2) {
        lv_obj_set_style_bg_color(g_soc_inner, COL_RED, 0);
        g_soc_alert = true;
    } else if (combined == 1) {
        lv_obj_set_style_bg_color(g_soc_inner, COL_YELLOW, 0);
    } else {
        lv_obj_set_style_bg_color(g_soc_inner, COL_GREEN, 0);
    }

    /* UDC label colour mirrors its own severity */
    g_udc_alert = false;
    if (udc_sev == 2) {
        lv_obj_set_style_text_color(g_udc_lbl, COL_RED, 0);
        g_udc_alert = true;
    } else if (udc_sev == 1) {
        lv_obj_set_style_text_color(g_udc_lbl, COL_YELLOW, 0);
    } else {
        lv_obj_set_style_text_color(g_udc_lbl, COL_RED, 0);
    }

    /* Temperature: highest severity wins; BAT is default/preferred */
    typedef struct { const char *n; float c; const thresh_t *th; } src_t;
    src_t srcs[4] = {
        {"BAT", t->temp_bat_c, &TH_BAT},
        {"MOT", t->temp_mot_c, &TH_MOT},
        {"INV", t->temp_inv_c, &TH_INV},
        {"ISA", t->temp_isa_c, &TH_ISA},
    };

    int best_sev     = severity(t->temp_bat_c, &TH_BAT);
    float best_c     = t->temp_bat_c;
    const char *best = (best_sev > 0) ? "BAT" : "---";

    if (best_sev == 0) {
        for (int i = 1; i < 4; i++) {
            int sv = severity(srcs[i].c, srcs[i].th);
            if (sv > best_sev) { best_sev = sv; best_c = srcs[i].c; best = srcs[i].n; }
        }
    }

    g_temp_alert = false;
    if (best_sev == 2) {
        lv_obj_set_style_bg_color(g_temp_inner, COL_RED, 0);
        g_temp_alert = true;
    } else if (best_sev == 1) {
        lv_obj_set_style_bg_color(g_temp_inner, COL_YELLOW, 0);
    } else {
        lv_obj_set_style_bg_color(g_temp_inner, COL_GREEN, 0);
    }

    /* Fire alert sound on the rising edge (first moment it goes red) */
    if (g_temp_alert && !s_prev_temp_alert && s_alert_cb)
        s_alert_cb();
    s_prev_temp_alert = g_temp_alert;

    if (best_sev > 0) {
        snprintf(buf, sizeof(buf), "%dC", (int)best_c);
        lv_label_set_text(g_temp_val_lbl, buf);
        lv_obj_clear_flag(g_temp_val_lbl, LV_OBJ_FLAG_HIDDEN);

        lv_label_set_text(g_temp_src_lbl, best);
        lv_obj_clear_flag(g_temp_src_lbl, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(g_temp_val_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(g_temp_src_lbl, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ─── gauge_ui_update_mileage ───────────────────────────────────────────── */

void gauge_ui_update_mileage(double total_miles, double trip_miles)
{
    char buf[48];
    int trip_int = (int)trip_miles;
    int trip_dec = (int)(trip_miles * 10) % 10;
    if (trip_dec < 0) trip_dec = -trip_dec;
    snprintf(buf, sizeof(buf), "ODO: %d\nTRP: %d.%d", (int)total_miles, trip_int, trip_dec);
    lv_label_set_text(g_odo_lbl, buf);
}

/* ─── gauge_ui_set_trip_cb ───────────────────────────────────────────────── */

void gauge_ui_set_trip_cb(void (*cb)(void))
{
    s_trip_cb = cb;
}

void gauge_ui_set_sound_cb(void (*cb)(void))
{
    s_sound_cb = cb;
}

void gauge_ui_set_alert_sound_cb(void (*cb)(void))
{
    s_alert_cb = cb;
}

/* ─── gauge_ui_blink_tick ───────────────────────────────────────────────── */

void gauge_ui_blink_tick(void)
{
    g_blink_on = !g_blink_on;
    lv_color_t on = COL_RED, off = COL_BG;

    if (g_soc_alert)
        lv_obj_set_style_bg_color(g_soc_inner,  g_blink_on ? on : off, 0);
    if (g_temp_alert)
        lv_obj_set_style_bg_color(g_temp_inner, g_blink_on ? on : off, 0);
    if (g_udc_alert)
        lv_obj_set_style_text_color(g_udc_lbl,  g_blink_on ? on : off, 0);
}
