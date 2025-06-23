#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <LovyanGFX.hpp> // Required for LGFX base class and types
#include "ui.h"
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <esp_wifi.h> // Required for esp_wifi_stop() and related functions
#include <esp_sleep.h>
#include <esp_bt.h>

const uint16_t kIrLedPin = 10;
IRsend irsend(kIrLedPin); // Initialize IR sender with the chosen pin

// Screen dimensions - these should match your display
static const uint16_t screenWidth = 320; // Example, adjust if needed
static const uint16_t screenHeight = 480; // Example, adjust if needed
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10]; // LVGL buffer

// Energy Saving Configuration
const int TOUCH_IRQ_PIN = 7; // GPIO7 for touch interrupt
const uint32_t INACTIVITY_TIMEOUT_MS = 20000; // 20 seconds
static unsigned long last_activity_time;
static bool screen_is_off = false;
static uint8_t last_brightness_before_off;

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_Parallel8 _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_FT5x06 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.freq_write = 40000000;
      cfg.pin_wr = 47;
      cfg.pin_rd = -1;
      cfg.pin_rs = 0;
      cfg.pin_d0 = 9;
      cfg.pin_d1 = 46;
      cfg.pin_d2 = 3;
      cfg.pin_d3 = 8;
      cfg.pin_d4 = 18;
      cfg.pin_d5 = 17;
      cfg.pin_d6 = 16;
      cfg.pin_d7 = 15;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = -1;
      cfg.pin_rst = 4;
      cfg.pin_busy = -1;
      cfg.memory_width = 320;
      cfg.memory_height = 480;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;

      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 45;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    {
      auto cfg = _touch_instance.config();
      cfg.i2c_port = 1;
      cfg.i2c_addr = 0x38;
      cfg.pin_sda = 6;
      cfg.pin_scl = 5;
      cfg.freq = 400000;
      cfg.x_min = 0;
      cfg.x_max = 320;
      cfg.y_min = 0;
      cfg.y_max = 480;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

LGFX lcd; // Declare an instance of your LGFX class AFTER the class definition

// This is the preferred display flush function using DMA

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    if (lcd.getStartCount() == 0) { // Processing if not yet started
        lcd.startWrite();
    }
    lcd.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::rgb565_t *)&color_p->full);
    lv_disp_flush_ready(disp);
}

// Add this helper function somewhere accessible
void wake_screen_if_off() {
    if (screen_is_off) {
        lcd.setBrightness(last_brightness_before_off);
        screen_is_off = false;
        Serial.println("Screen on"); // Generic message, or pass a source string
        // Invalidate the whole screen to force a redraw when waking up
        // This is good practice to ensure the UI is fully refreshed.
        if(lv_disp_get_default()) {
            lv_obj_invalidate(lv_scr_act());
        }
    }
    last_activity_time = millis(); // Always update activity time when interaction occurs
}

// Then in your touchpad read:
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t touchX = 0, touchY = 0;
    bool touched = lcd.getTouch(&touchX, &touchY);
    
    if (touched) {
        wake_screen_if_off(); // Call the helper
        // Serial.println("Screen on (touch)"); // Specific log if needed, or integrate into wake_screen_if_off
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touchX;
        data->point.y = touchY;
    } else {
        if (data->state == LV_INDEV_STATE_PR) { // Was pressed, now released
             last_activity_time = millis(); // Update on release as well
        }
       data->state = LV_INDEV_STATE_REL;
    }
}

// --- IR Sending Configuration for different SIRC Protocols ---
// Common repeats for Sony, adjust if needed
const uint8_t SONY_DEFAULT_REPEATS = 3;
const uint8_t SONY_SIRC15_BITS = 15;

// These functions can be called from ui_events.c or other C code.
// They take the 7-bit command code for the respective SONY SIRC protocol.
#ifdef __cplusplus
extern "C" {
#endif

void send_sirc15_command(uint32_t device_address_8bit, uint32_t command_code_7bit) {
    uint8_t addr = device_address_8bit & 0xFF; // Ensure address is 8 bits
    uint8_t cmd = command_code_7bit & 0x7F; // Ensure command is 7 bits
    // SIRC15: 8-bit address, 7-bit command
    uint16_t data = (addr << 7) | cmd;

    LV_LOG_USER("Sending SONY SIRC15: Addr=0x%X, Cmd=0x%X, Data=0x%X, Bits=%d, Repeats=%d",
                addr,
                cmd,
                data,
                SONY_SIRC15_BITS,
                SONY_DEFAULT_REPEATS);
    irsend.sendSony(data, SONY_SIRC15_BITS, SONY_DEFAULT_REPEATS);
}

#ifdef __cplusplus
}
#endif

void setup() {
    Serial.begin(115200); /* prepare for possible serial debug */

    // Disable Wi-Fi if not used to save power
    // esp_wifi_stop() should be called after Wi-Fi has been initialized by the system,
    // which is usually the case by the time setup() runs in Arduino.
    //esp_wifi_stop();
    WiFi.mode(WIFI_OFF);
    Serial.println("Wi-Fi stopped.");

    btStop(); // Disable Bluetooth
    esp_bt_controller_disable();
    Serial.println("Bluetooth stopped.");

    irsend.begin(); // Initializes the IR sender

    String LVGL_Arduino = "Sony LAM-Z05 IR";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);

    // Pin init
    // LCD_CS and LCD_BLK are typically handled by LovyanGFX based on your LGFX class config.
    // If cfg.pin_cs is -1, CS might be tied low or handled by the bus.
    // cfg.pin_bl (pin 45) will be handled for backlight.
    // The I2C for touch (port 1, pins 6, 5) is also handled by LovyanGFX.
    // If you have other I2C devices on Wire (port 0), initialize it separately with its specific pins.
    // Initialize LCD (LGFX)
    lcd.init();
    lcd.setRotation(2); // Set your rotation
    last_brightness_before_off = lcd.getBrightness();
    if (last_brightness_before_off == 0 && lcd.getPanel()->getLight() != nullptr) { // If backlight was initially off but exists
        last_brightness_before_off = 255; // Default to full brightness
    }
    // The touch interrupt pin (GPIO7 for WT32-SC01 Plus) is typically handled by the
    // touch driver (_touch_instance) if it uses interrupts for normal operation.

    // Configure GPIO7 as wake-up source (FT5x06 INT is active LOW)
    pinMode(TOUCH_IRQ_PIN, INPUT_PULLUP); // Use internal pull-up for safety if INT is open-drain
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0); // Wake up when GPIO7 is LOW

    lv_init();

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10); // LVGL buffer

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*Initialize the input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;

    // Optional: Adjust the long-press time (default is 400ms in LVGL v8).
    // This must be set *before* registering the driver.
    indev_drv.long_press_time = 800; // e.g., set to 800ms

    lv_indev_drv_register(&indev_drv);
    ui_init();

    last_activity_time = millis(); // Initialize activity timer
    screen_is_off = false; // Screen starts on

    Serial.println("Setup done");
}

void loop() {
    if (!screen_is_off) { // If screen is currently on
        if (millis() - last_activity_time > INACTIVITY_TIMEOUT_MS) {
            Serial.println("Inactivity timeout. Preparing for light sleep.");
            
            // Save current brightness and turn off screen
            if (lcd.getPanel()->getLight() != nullptr) {
                uint8_t current_brightness = lcd.getBrightness();
                if (current_brightness > 0) { // Only save if it was actually on
                    last_brightness_before_off = current_brightness;
                } else if (last_brightness_before_off == 0) { // If it was already 0, ensure we restore to a default
                    last_brightness_before_off = 255;
                }
                lcd.setBrightness(0);
            }
            screen_is_off = true; // Mark screen as logically off

            Serial.println("Entering light sleep...");
            esp_light_sleep_start();
            // ---- Execution resumes here after wake-up by touch IRQ on GPIO7 ----
            Serial.println("Woke up from light sleep!");
            
            wake_screen_if_off(); // Restore screen brightness, reset flags, and update activity timer
        }
    }

    lv_timer_handler();
    delay(5); // A smaller delay like 5ms can improve responsiveness
}
