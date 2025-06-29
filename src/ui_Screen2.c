// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

lv_obj_t * uic_servmenu;
lv_obj_t * uic_vol_plus;
lv_obj_t * uic_vol_minus;
lv_obj_t * uic_dspl;
lv_obj_t * uic_resume;
lv_obj_t * uic_snd;
lv_obj_t * uic_spd_high;
lv_obj_t * uic_spd_norm;
lv_obj_t * uic_rec_mode;
lv_obj_t * uic_mtrx_sur;
lv_obj_t * uic_grp_plus;
lv_obj_t * uic_grp_min;
lv_obj_t * uic_grp_mode;
lv_obj_t * uic_ntwk;
lv_obj_t * uic_stop;
lv_obj_t * uic_cd_pl;
lv_obj_t * uic_frwd;
lv_obj_t * uic_bkw;
lv_obj_t * uic_md_pl;
lv_obj_t * ui_Screen2;
lv_obj_t * ui_Image5;
lv_obj_t * ui_Button4;
lv_obj_t * ui_Panel25;
lv_obj_t * ui_Panel24;
lv_obj_t * ui_Panel26;
lv_obj_t * ui_Panel27;
lv_obj_t * ui_Panel28;
lv_obj_t * ui_Panel29;
lv_obj_t * ui_Panel30;
lv_obj_t * ui_Panel31;
lv_obj_t * ui_Panel32;
lv_obj_t * ui_Panel33;
lv_obj_t * ui_Panel34;
lv_obj_t * ui_Panel35;
lv_obj_t * ui_Panel36;
lv_obj_t * ui_Panel37;
lv_obj_t * ui_Panel38;
lv_obj_t * ui_Panel39;
lv_obj_t * ui_Panel41;
lv_obj_t * ui_Panel40;
lv_obj_t * ui_Button3 = NULL;
lv_obj_t * ui_Label1 = NULL;

// event funtions
void ui_event_Button4(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Screen1_screen_init);
    }
}

void ui_event_Button3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_LONG_PRESSED) {
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 500, 3, &ui_Screen3_screen_init);
    }
}
// build funtions

void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x5B5B5B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image5 = lv_img_create(ui_Screen2);
    lv_img_set_src(ui_Image5, &ui_img_screenshot_51_png);
    lv_obj_set_width(ui_Image5, LV_SIZE_CONTENT);   /// 311
    lv_obj_set_height(ui_Image5, LV_SIZE_CONTENT);    /// 410
    lv_obj_set_x(ui_Image5, 0);
    lv_obj_set_y(ui_Image5, 29);
    lv_obj_set_align(ui_Image5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image5, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button4 = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Button4, 144);
    lv_obj_set_height(ui_Button4, 50);
    lv_obj_set_x(ui_Button4, 80);
    lv_obj_set_y(ui_Button4, -209);
    lv_obj_set_align(ui_Button4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button4, lv_color_hex(0x175834), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Button4, lv_color_hex(0x0A2B0D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Button4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Button4, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Button4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Button4, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Button4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Button4, lv_color_hex(0x02E71B), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Button4, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Button4, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Button4, 0, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel25 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel25, 40);
    lv_obj_set_height(ui_Panel25, 40);
    lv_obj_set_x(ui_Panel25, -108);
    lv_obj_set_y(ui_Panel25, -126);
    lv_obj_set_align(ui_Panel25, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel25, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel25, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel25, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel25, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel25, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel25, 30, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel25, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel25, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel25, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel25, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel25, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel25, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel24 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel24, 40);
    lv_obj_set_height(ui_Panel24, 40);
    lv_obj_set_x(ui_Panel24, -36);
    lv_obj_set_y(ui_Panel24, -126);
    lv_obj_set_align(ui_Panel24, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel24, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel24, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel24, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel24, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel24, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel24, 30, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel24, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel24, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel24, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel24, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel24, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel24, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel26 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel26, 40);
    lv_obj_set_height(ui_Panel26, 40);
    lv_obj_set_x(ui_Panel26, 35);
    lv_obj_set_y(ui_Panel26, -127);
    lv_obj_set_align(ui_Panel26, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel26, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel26, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel26, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel26, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel26, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel26, 30, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel26, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel26, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel26, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel26, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel26, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel26, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel27 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel27, 40);
    lv_obj_set_height(ui_Panel27, 40);
    lv_obj_set_x(ui_Panel27, -107);
    lv_obj_set_y(ui_Panel27, -58);
    lv_obj_set_align(ui_Panel27, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel27, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel27, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel27, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel27, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel27, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel27, 30, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel27, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel27, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel27, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel27, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel27, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel27, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel28 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel28, 40);
    lv_obj_set_height(ui_Panel28, 40);
    lv_obj_set_x(ui_Panel28, -36);
    lv_obj_set_y(ui_Panel28, -59);
    lv_obj_set_align(ui_Panel28, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel28, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel28, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel28, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel28, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel28, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel28, 30, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel28, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel28, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel28, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel28, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel28, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel28, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel29 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel29, 40);
    lv_obj_set_height(ui_Panel29, 40);
    lv_obj_set_x(ui_Panel29, 36);
    lv_obj_set_y(ui_Panel29, -59);
    lv_obj_set_align(ui_Panel29, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel29, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel29, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel29, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel29, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel29, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel29, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel29, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel29, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel29, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel29, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel29, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel29, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel30 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel30, 40);
    lv_obj_set_height(ui_Panel30, 40);
    lv_obj_set_x(ui_Panel30, -107);
    lv_obj_set_y(ui_Panel30, 10);
    lv_obj_set_align(ui_Panel30, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel30, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel30, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel30, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel30, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel30, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel30, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel30, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel30, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel30, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel30, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel30, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel30, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel31 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel31, 40);
    lv_obj_set_height(ui_Panel31, 40);
    lv_obj_set_x(ui_Panel31, -36);
    lv_obj_set_y(ui_Panel31, 10);
    lv_obj_set_align(ui_Panel31, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel31, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel31, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel31, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel31, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel31, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel31, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel31, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel31, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel31, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel31, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel31, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel31, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel32 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel32, 40);
    lv_obj_set_height(ui_Panel32, 40);
    lv_obj_set_x(ui_Panel32, 36);
    lv_obj_set_y(ui_Panel32, 10);
    lv_obj_set_align(ui_Panel32, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel32, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel32, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel32, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel32, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel32, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel32, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel32, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel32, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel32, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel32, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel32, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel32, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel33 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel33, 40);
    lv_obj_set_height(ui_Panel33, 40);
    lv_obj_set_x(ui_Panel33, 108);
    lv_obj_set_y(ui_Panel33, 10);
    lv_obj_set_align(ui_Panel33, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel33, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel33, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel33, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel33, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel33, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel33, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel33, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel33, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel33, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel33, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel33, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel33, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel34 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel34, 40);
    lv_obj_set_height(ui_Panel34, 40);
    lv_obj_set_x(ui_Panel34, -107);
    lv_obj_set_y(ui_Panel34, 78);
    lv_obj_set_align(ui_Panel34, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel34, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel34, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel34, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel34, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel34, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel34, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel34, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel34, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel34, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel34, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel34, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel34, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel35 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel35, 40);
    lv_obj_set_height(ui_Panel35, 40);
    lv_obj_set_x(ui_Panel35, -35);
    lv_obj_set_y(ui_Panel35, 78);
    lv_obj_set_align(ui_Panel35, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel35, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel35, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel35, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel35, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel35, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel35, 30, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel35, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel35, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel35, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel35, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel35, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel35, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel36 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel36, 40);
    lv_obj_set_height(ui_Panel36, 40);
    lv_obj_set_x(ui_Panel36, 36);
    lv_obj_set_y(ui_Panel36, 78);
    lv_obj_set_align(ui_Panel36, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel36, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel36, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel36, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel36, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel36, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel36, 20, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel36, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel36, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel36, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel36, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel36, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel36, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel37 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel37, 40);
    lv_obj_set_height(ui_Panel37, 40);
    lv_obj_set_x(ui_Panel37, 108);
    lv_obj_set_y(ui_Panel37, 77);
    lv_obj_set_align(ui_Panel37, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel37, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel37, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel37, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel37, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel37, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel37, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel37, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel37, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel37, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel37, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel37, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel37, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel38 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel38, 40);
    lv_obj_set_height(ui_Panel38, 40);
    lv_obj_set_x(ui_Panel38, -106);
    lv_obj_set_y(ui_Panel38, 163);
    lv_obj_set_align(ui_Panel38, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel38, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel38, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel38, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel38, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel38, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel38, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel38, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel38, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel38, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel38, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel38, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel38, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel39 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel39, 40);
    lv_obj_set_height(ui_Panel39, 40);
    lv_obj_set_x(ui_Panel39, 108);
    lv_obj_set_y(ui_Panel39, 162);
    lv_obj_set_align(ui_Panel39, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel39, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel39, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_blend_mode(ui_Panel39, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_Panel39, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel39, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel39, 50, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel39, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel39, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel39, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel39, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel39, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel39, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel41 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel41, 40);
    lv_obj_set_height(ui_Panel41, 40);
    lv_obj_set_x(ui_Panel41, 108);
    lv_obj_set_y(ui_Panel41, -59);
    lv_obj_set_align(ui_Panel41, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel41, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel41, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_Panel41, &ui_img_minus_inact_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_opa(ui_Panel41, 230, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel41, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel41, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel41, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel41, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel41, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_img_src(ui_Panel41, &ui_img_minus_inact_png, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_img_opa(ui_Panel41, 200, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel41, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel41, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel41, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel41, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel41, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel41, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Panel40 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Panel40, 40);
    lv_obj_set_height(ui_Panel40, 40);
    lv_obj_set_x(ui_Panel40, 107);
    lv_obj_set_y(ui_Panel40, -126);
    lv_obj_set_align(ui_Panel40, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel40, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel40, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_Panel40, &ui_img_plus_inact_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_opa(ui_Panel40, 230, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel40, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel40, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel40, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel40, lv_color_hex(0x0462AF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_Panel40, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_Panel40, lv_color_hex(0x03F636), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_Panel40, 255, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_Panel40, 5, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_Panel40, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_blend_mode(ui_Panel40, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_opa(ui_Panel40, 255, LV_PART_MAIN | LV_STATE_PRESSED);

    ui_Button3 = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Button3, 40);
    lv_obj_set_height(ui_Button3, 40);
    lv_obj_set_x(ui_Button3, -132);
    lv_obj_set_y(ui_Button3, -213);
    lv_obj_set_align(ui_Button3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button3, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button3, lv_color_hex(0xA4A1A1), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button3, 120, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_Button3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_Button3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -132);
    lv_obj_set_y(ui_Label1, -213);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "SM");

    lv_obj_add_event_cb(ui_Button4, ui_event_Button4, LV_EVENT_ALL, NULL);
	lv_obj_add_event_cb(ui_Button3, ui_event_Button3, LV_EVENT_ALL, NULL);
    uic_md_pl = ui_Panel25;
    uic_bkw = ui_Panel24;
    uic_frwd = ui_Panel26;
    uic_cd_pl = ui_Panel27;
    uic_stop = ui_Panel28;
    uic_ntwk = ui_Panel29;
    uic_grp_mode = ui_Panel30;
    uic_grp_min = ui_Panel31;
    uic_grp_plus = ui_Panel32;
    uic_mtrx_sur = ui_Panel33;
    uic_rec_mode = ui_Panel34;
    uic_spd_norm = ui_Panel35;
    uic_spd_high = ui_Panel36;
    uic_snd = ui_Panel37;
    uic_resume = ui_Panel38;
    uic_dspl = ui_Panel39;
    uic_vol_minus = ui_Panel41;
    uic_vol_plus = ui_Panel40;
    uic_servmenu = ui_Button3;

    // Add event callbacks for Screen2 custom buttons
    lv_obj_add_event_cb(uic_md_pl, ui_event_uic_md_pl_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_bkw, ui_event_uic_bkw_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_frwd, ui_event_uic_frwd_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_cd_pl, ui_event_uic_cd_pl_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_stop, ui_event_uic_stop_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_ntwk, ui_event_uic_ntwk_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_grp_mode, ui_event_uic_grp_mode_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_grp_min, ui_event_uic_grp_min_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_grp_plus, ui_event_uic_grp_plus_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_mtrx_sur, ui_event_uic_mtrx_sur_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_rec_mode, ui_event_uic_rec_mode_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_spd_norm, ui_event_uic_spd_norm_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_spd_high, ui_event_uic_spd_high_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_snd, ui_event_uic_snd_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_resume, ui_event_uic_resume_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_dspl, ui_event_uic_dspl_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_vol_minus, ui_event_uic_vol_minus_clicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(uic_vol_plus, ui_event_uic_vol_plus_clicked, LV_EVENT_CLICKED, NULL);
    //lv_obj_add_event_cb(uic_servmenu, ui_event_uic_servmenu_clicked, LV_EVENT_LONG_PRESSED, NULL);

}

void ui_Screen2_screen_destroy(void)
{
    if(ui_Screen2) lv_obj_del(ui_Screen2);

    // NULL screen variables
    ui_Screen2 = NULL;
    ui_Image5 = NULL;
    ui_Button4 = NULL;
    uic_md_pl = NULL;
    ui_Panel25 = NULL;
    uic_bkw = NULL;
    ui_Panel24 = NULL;
    uic_frwd = NULL;
    ui_Panel26 = NULL;
    uic_cd_pl = NULL;
    ui_Panel27 = NULL;
    uic_stop = NULL;
    ui_Panel28 = NULL;
    uic_ntwk = NULL;
    ui_Panel29 = NULL;
    uic_grp_mode = NULL;
    ui_Panel30 = NULL;
    uic_grp_min = NULL;
    ui_Panel31 = NULL;
    uic_grp_plus = NULL;
    ui_Panel32 = NULL;
    uic_mtrx_sur = NULL;
    ui_Panel33 = NULL;
    uic_rec_mode = NULL;
    ui_Panel34 = NULL;
    uic_spd_norm = NULL;
    ui_Panel35 = NULL;
    uic_spd_high = NULL;
    ui_Panel36 = NULL;
    uic_snd = NULL;
    ui_Panel37 = NULL;
    uic_resume = NULL;
    ui_Panel38 = NULL;
    uic_dspl = NULL;
    ui_Panel39 = NULL;
    uic_vol_minus = NULL;
    ui_Panel41 = NULL;
    uic_vol_plus = NULL;
    ui_Panel40 = NULL;
    uic_servmenu = NULL;
    ui_Button3 = NULL;
    ui_Label1 = NULL;

}
