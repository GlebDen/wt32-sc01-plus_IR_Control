SET(SOURCES ui_Screen1.c
    ui_Screen2.c
    ui_Screen3.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_img_screenshot_50_png.c
    ui_img_screenshot_51_png.c
    ui_img_minus_inact_png.c
    ui_img_plus_inact_png.c)

add_library(ui ${SOURCES})
