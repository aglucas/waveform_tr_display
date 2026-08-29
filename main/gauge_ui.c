#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "gauge_ui.h"
#include "triumph_logo.h"

LV_FONT_DECLARE(lv_font_montserrat_bold_28)
LV_FONT_DECLARE(lv_font_montserrat_bold_36)
LV_FONT_DECLARE(lv_font_montserrat_bold_44)
/* Value-only font for the large readout: digits plus ".-%CVW" and space. The
 * title line above it uses the 36 px font (full ASCII), which keeps this one
 * to 16 glyphs — a full-ASCII 88 px face would cost ~900 kB of flash. */
LV_FONT_DECLARE(lv_font_montserrat_bold_88)

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
#define NEEDLE_WIDTH      12   /* stroke thickness, px                  */
#define TICK_MINOR_LEN   13
#define TICK_MAJOR_LEN   22

/* Warning indicator positions (scaled ~1.67× from 480-px CircuitPython) */
#define SOC_X            200
#define TEMP_X           600
#define WARN_Y           CY
#define WARN_OUTER_R     30
#define WARN_INNER_R     22
#define WARN_ROW_H       50    /* vertical gap between label rows       */

/* Large readout — sits under the 50 mph point (50 of 100 on a 300 deg sweep from
 * 7 o'clock lands at 12 o'clock, i.e. straight up). It overlaps the needle's
 * travel near 50 mph, so it's backed by an opaque plaque and created last, which
 * keeps it legible on top rather than fighting the needle for contrast. */
#define BIG_CY           200   /* plaque centre                          */
#define BIG_W            340
#define BIG_H            180
#define BIG_TITLE_Y      150
#define BIG_VAL_Y        232

/* 12V aux battery readout — the last slot in the SOC column before it runs
 * into the ODO/TRP plaque (top edge at ODO_Y - 60 = 620), hence the small
 * font: at line_height 28 the label's bottom edge (606 + 14 = 620) lands
 * exactly on the plaque with no overlap. */
#define AUX_Y            606

#define ODO_Y            680
/* Car-mode readout, just below the ODO/TRP plaque (which spans y 620..740).
 * Kept outside the plaque so it doesn't enlarge the trip-reset tap target.
 * The gauge sweep leaves a 60 deg gap at 6 o'clock, so this area is clear. */
#define MODE_Y           760

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
static lv_obj_t *g_hub;
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
static lv_obj_t *g_mode_lbl;
static lv_obj_t *g_whmi_lbl;
static lv_obj_t *g_aux_lbl;
static lv_obj_t *g_big_bg;
static lv_obj_t *g_big_title;
static lv_obj_t *g_big_val;

/* Trip reset callback */
static void (*s_trip_cb)(void) = NULL;

static void odo_click_cb(lv_event_t *e)
{
    if (s_trip_cb) s_trip_cb();
}

/* Sound callbacks — tap uses normal volume, alert uses full volume */
static void (*s_sound_cb)(void)       = NULL;
static void (*s_alert_cb)(void)       = NULL;
static bool s_prev_temp_alert = false;

static void temp_click_cb(lv_event_t *e)
{
    if (s_sound_cb) s_sound_cb();
}

/* Brightness step callback — pivot hub tapped */
static void (*s_brightness_cb)(void)  = NULL;

static void hub_click_cb(lv_event_t *e)
{
    ESP_LOGI("gauge_ui", "Hub dot pressed");
    if (s_brightness_cb) s_brightness_cb();
}

/* Wh/mile readout — three meters, cycled by tapping the large readout:
 *   T = since trip reset      C = since last charge      M = last 10 miles
 * M is trip-based (main.c hands us the window deltas), so it resets with T. */
typedef enum {
    WHMI_TRIP = 0,
    WHMI_CHARGE,
    WHMI_ROLL,
    WHMI_MODE_COUNT,
} whmi_mode_t;

static double s_whmi_trip_wh = 0.0, s_whmi_trip_mi = 0.0;
static double s_whmi_chg_wh  = 0.0, s_whmi_chg_mi  = 0.0;
static double s_whmi_roll_wh = 0.0, s_whmi_roll_mi = 0.0;
static whmi_mode_t s_whmi_mode = WHMI_TRIP;

/* The (wh, mi) pair and tag letter for the currently selected meter. */
static void whmi_selected(double *wh, double *mi, char *tag)
{
    switch (s_whmi_mode) {
        case WHMI_CHARGE: *wh = s_whmi_chg_wh;  *mi = s_whmi_chg_mi;  *tag = 'C'; break;
        case WHMI_ROLL:   *wh = s_whmi_roll_wh; *mi = s_whmi_roll_mi; *tag = 'M'; break;
        default:          *wh = s_whmi_trip_wh; *mi = s_whmi_trip_mi; *tag = 'T'; break;
    }
}

/* Below ~0.1 mi the ratio is noise-dominated, so hold at 0. fabs() makes the
 * reading polarity-independent (the energy counter's sign depends on wiring). */
static int whmi_value(double wh, double mi)
{
    if (mi < 0.1) return 0;
    double v = fabs(wh) / mi;
    if (v > 999.0) v = 999.0;   /* keep it to 3 digits */
    return (int)(v + 0.5);
}

static void whmi_render(void)
{
    char buf[24];
    double wh, mi;
    char   tag;
    whmi_selected(&wh, &mi, &tag);
    snprintf(buf, sizeof(buf), "%c %d\nWh/mi", tag, whmi_value(wh, mi));
    lv_label_set_text(g_whmi_lbl, buf);
}

/* ─── Large readout ──────────────────────────────────────────────────────────
 * One value at a time, promoted by tapping its small label. Tapping the small
 * label of the value already shown demotes it (clears the area); tapping a
 * different one swaps it in. The latest telemetry is cached here so the big
 * readout can re-render on demand without waiting for the next frame. */
typedef enum {
    BIG_NONE = 0,
    BIG_SOC,
    BIG_BAT,
    BIG_UDC,
    BIG_WHMI,
} big_item_t;

static big_item_t s_big_item = BIG_NONE;
static float s_soc_pct = 0.0f, s_bat_c = 0.0f, s_udc_v = 0.0f;

/* Value strings must stay within the 88 px font's glyph set (digits .-%CVW). */
static void big_render(void)
{
    if (s_big_item == BIG_NONE) {
        lv_obj_add_flag(g_big_bg,    LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(g_big_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(g_big_val,   LV_OBJ_FLAG_HIDDEN);
        return;
    }

    char title[16], val[12];
    switch (s_big_item) {
        case BIG_SOC:
            snprintf(title, sizeof(title), "SOC");
            snprintf(val,   sizeof(val),   "%d%%", (int)s_soc_pct);
            break;
        case BIG_BAT:
            snprintf(title, sizeof(title), "BAT TEMP");
            snprintf(val,   sizeof(val),   "%dC", (int)s_bat_c);
            break;
        case BIG_UDC:
            snprintf(title, sizeof(title), "DC BUS");
            snprintf(val,   sizeof(val),   "%dV", (int)s_udc_v);
            break;
        case BIG_WHMI: {
            double wh, mi;
            char   tag;
            whmi_selected(&wh, &mi, &tag);
            const char *name = (s_whmi_mode == WHMI_CHARGE) ? "CHG Wh/mi"
                             : (s_whmi_mode == WHMI_ROLL)   ? "LAST 10 MI"
                                                            : "TRIP Wh/mi";
            snprintf(title, sizeof(title), "%s", name);
            snprintf(val,   sizeof(val),   "%d", whmi_value(wh, mi));
            break;
        }
        default:
            return;
    }

    lv_label_set_text(g_big_title, title);
    lv_label_set_text(g_big_val,   val);
    lv_obj_clear_flag(g_big_bg,    LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_big_title, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_big_val,   LV_OBJ_FLAG_HIDDEN);
}

/* Tapping the small label: promote it, or demote if it's the one already up. */
static void big_toggle(big_item_t item)
{
    s_big_item = (s_big_item == item) ? BIG_NONE : item;
    big_render();
}

/* All of these run in LVGL task context (under the display lock), so touching
 * widgets directly is safe here — no flag/hand-off needed. */
static void soc_lbl_click_cb(lv_event_t *e) { big_toggle(BIG_SOC); }
static void bat_lbl_click_cb(lv_event_t *e) { big_toggle(BIG_BAT); }
static void udc_lbl_click_cb(lv_event_t *e) { big_toggle(BIG_UDC); }
static void whmi_click_cb   (lv_event_t *e) { big_toggle(BIG_WHMI); }

/* Tapping the large readout itself: for Wh/mi it cycles Trip -> Charge -> last
 * 10 mi -> Trip (the small label follows); for anything else there's nothing to
 * cycle, so it just clears the area. */
static void big_click_cb(lv_event_t *e)
{
    if (s_big_item == BIG_WHMI) {
        s_whmi_mode = (whmi_mode_t)((s_whmi_mode + 1) % WHMI_MODE_COUNT);
        whmi_render();   /* keep the small label in sync */
        big_render();
    } else {
        s_big_item = BIG_NONE;
        big_render();
    }
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
/* Car opmode → display text + colour. Values outside the known set are already
 * rejected by telemetry_valid(), so the default is a defensive fallback only. */
static const char *mode_text(uint8_t opmode, lv_color_t *col)
{
    switch ((car_mode_t)opmode) {
        case CAR_MODE_RUN:       *col = COL_GREEN;  return "RUN";
        case CAR_MODE_CHARGE:    *col = COL_GREEN;  return "CHARGING";
        case CAR_MODE_PRECHARGE: *col = COL_YELLOW; return "PRECHARGE";
        case CAR_MODE_PREHEAT:   *col = COL_YELLOW; return "12V Maint";
        case CAR_MODE_PCHFAIL:   *col = COL_RED;    return "PCH FAIL";
        case CAR_MODE_OFF:       *col = COL_WHITE;  return "OFF";
        default:                 *col = COL_WHITE;  return "---";
    }
}

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
    lv_style_set_text_color(&s_ticks_major, COL_WHITE);
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
    lv_style_set_line_width(&s_needle_line, NEEDLE_WIDTH);
    lv_style_set_line_rounded(&s_needle_line, true);

    g_needle = lv_line_create(g_scale);
    lv_obj_add_style(g_needle, &s_needle_line, 0);
    lv_scale_set_line_needle_value(g_scale, g_needle, NEEDLE_LEN, 0);

    /* Hub dot — clickable for brightness control; color reports whether the
     * odometer save on disk was a confirmed PLP save (see gauge_ui_set_hub_ok). */
    g_hub = circle(parent, CX, CY, WARN_OUTER_R, COL_RED);
    lv_obj_add_flag(g_hub, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_hub, hub_click_cb, LV_EVENT_CLICKED, NULL);

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
                                 "20C",  COL_WHITE, df);
    g_soc_pct_lbl  = fixed_label(parent, SOC_X, WARN_Y - WARN_ROW_H * 2, 150, 1,
                                 "100%", COL_WHITE, df);
    g_udc_lbl      = fixed_label(parent, SOC_X, WARN_Y - WARN_ROW_H,     150, 1,
                                 "---",  COL_WHITE, df);

    /* Each small value promotes itself into the large readout when tapped. */
    lv_obj_add_flag(g_bat_temp_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_bat_temp_lbl, bat_lbl_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(g_soc_pct_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_soc_pct_lbl, soc_lbl_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(g_udc_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_udc_lbl, udc_lbl_click_cb, LV_EVENT_CLICKED, NULL);

    /* Efficiency (Wh/mile) — under the SOC block, in the gap above the ODO
     * plaque. Two lines: the tag+3-digit value ("T 250") over a "Wh/mi" unit
     * label. Tappable to toggle between the Trip and Charge meters. */
    g_whmi_lbl = fixed_label(parent, SOC_X, WARN_Y + WARN_ROW_H * 3, 200, 2,
                             "T 0\nWh/mi", COL_WHITE, &lv_font_montserrat_bold_36);
    lv_obj_add_flag(g_whmi_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_whmi_lbl, whmi_click_cb, LV_EVENT_CLICKED, NULL);

    /* 12V aux battery voltage — last readout in the SOC column */
    g_aux_lbl = fixed_label(parent, SOC_X, AUX_Y, 150, 1,
                            "--.-V", COL_WHITE, &lv_font_montserrat_bold_28);

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
                            "ODO: 0\nTRP: 0.0", COL_WHITE, &lv_font_montserrat_bold_36);

    /* Car mode (Off / Run / Charging / …) directly under the TRP line */
    g_mode_lbl = fixed_label(parent, CX, MODE_Y, 280, 1,
                             "---", COL_WHITE, &lv_font_montserrat_bold_28);

    /* ── Large readout — created LAST so it draws over the needle ──────────── */
    g_big_bg = lv_obj_create(parent);
    lv_obj_set_size(g_big_bg, BIG_W, BIG_H);
    lv_obj_set_pos(g_big_bg, CX - BIG_W / 2, BIG_CY - BIG_H / 2);
    lv_obj_set_style_bg_color(g_big_bg, COL_BLACK, 0);
    lv_obj_set_style_bg_opa(g_big_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_big_bg, COL_RED, 0);
    lv_obj_set_style_border_width(g_big_bg, 3, 0);
    lv_obj_set_style_radius(g_big_bg, 10, 0);
    lv_obj_clear_flag(g_big_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_big_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_big_bg, big_click_cb, LV_EVENT_CLICKED, NULL);

    g_big_title = fixed_label(parent, CX, BIG_TITLE_Y, BIG_W - 20, 1,
                              "SOC", COL_WHITE, &lv_font_montserrat_bold_36);
    g_big_val   = fixed_label(parent, CX, BIG_VAL_Y,   BIG_W - 20, 1,
                              "0", COL_WHITE, &lv_font_montserrat_bold_88);

    /* The two labels are left non-clickable (the default), so LVGL's hit test
     * skips them and the tap lands on the plaque underneath — that's what makes
     * the whole readout, text included, one tap target for big_click_cb. */

    big_render();   /* starts hidden (BIG_NONE) */
}

/* ─── gauge_ui_update_telemetry ─────────────────────────────────────────── */

void gauge_ui_update_telemetry(const telemetry_t *t)
{
    char buf[32];

    /* The pack contactors are only closed in Run or Charge. In any other opmode
     * the DC bus isn't energised and SOC isn't meaningful — the sender sends
     * placeholder values (DEFAULT_UDC_V / DEFAULT_SOC_PCT) rather than a real
     * reading. Judging those placeholders is what caused false warnings: the
     * 400 V default sits exactly on TH_UDC's warning threshold (hw = 400). So
     * gate both the readout and the severity on this one flag. */
    bool bus_live = (t->opmode == CAR_MODE_RUN || t->opmode == CAR_MODE_CHARGE);

    /* Cache for the large readout, which re-renders from these on demand. */
    s_soc_pct = t->soc_pct;
    s_bat_c   = t->temp_bat_c;
    s_udc_v   = bus_live ? t->udc_v : 0.0f;   /* big readout shows 0 V too */

    /* Needle */
    int spd = (int)(t->speed_mph < 0 ? 0 : t->speed_mph > MAX_SPEED
                    ? MAX_SPEED : t->speed_mph);
    lv_scale_set_line_needle_value(g_scale, g_needle, NEEDLE_LEN, spd);

    /* DC bus voltage label — 0 V when the bus isn't live (no reading to show) */
    snprintf(buf, sizeof(buf), "%dV", bus_live ? (int)t->udc_v : 0);
    lv_label_set_text(g_udc_lbl, buf);

    /* SOC % label */
    snprintf(buf, sizeof(buf), "%d%%", (int)t->soc_pct);
    lv_label_set_text(g_soc_pct_lbl, buf);

    /* Car mode label */
    lv_color_t mode_col;
    const char *mode_txt = mode_text(t->opmode, &mode_col);
    lv_label_set_text(g_mode_lbl, mode_txt);
    lv_obj_set_style_text_color(g_mode_lbl, mode_col, 0);

    /* Battery temperature label */
    snprintf(buf, sizeof(buf), "%dC", (int)t->temp_bat_c);
    lv_label_set_text(g_bat_temp_lbl, buf);

    /* 12V aux battery voltage */
    snprintf(buf, sizeof(buf), "%.1fV", t->uaux_v);
    lv_label_set_text(g_aux_lbl, buf);

    /* SOC indicator: worst of SOC% and bus voltage. Both are only meaningful
     * with the contactors closed (Run or Charge); outside that we're looking at
     * placeholders, so force severity to 0 rather than grading a made-up number.
     * A 0 V bus is the expected resting state, not a fault — hence no warning. */
    int soc_sev = (!bus_live) ? 0
                : (t->soc_pct < SOC_ALERT_PCT) ? 2
                : (t->soc_pct < SOC_WARN_PCT)  ? 1 : 0;
    int udc_sev = (!bus_live) ? 0 : severity(t->udc_v, &TH_UDC);
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
        lv_obj_set_style_text_color(g_udc_lbl, COL_WHITE, 0);
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

    big_render();   /* keep the promoted value live */
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

/* ─── gauge_ui_update_energy ────────────────────────────────────────────── */

void gauge_ui_update_energy(double trip_wh, double trip_miles,
                            double charge_wh, double charge_miles,
                            double roll_wh, double roll_miles)
{
    s_whmi_trip_wh = trip_wh;   s_whmi_trip_mi = trip_miles;
    s_whmi_chg_wh  = charge_wh; s_whmi_chg_mi  = charge_miles;
    s_whmi_roll_wh = roll_wh;   s_whmi_roll_mi = roll_miles;
    whmi_render();   /* honours the current T/C/M selection */
    big_render();    /* keep the promoted value live if it's Wh/mi */
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

void gauge_ui_set_brightness_cb(void (*cb)(void))
{
    s_brightness_cb = cb;
}

void gauge_ui_set_hub_ok(bool ok)
{
    lv_obj_set_style_bg_color(g_hub, ok ? COL_GREEN : COL_RED, 0);
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
