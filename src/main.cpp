// define the following to set the clock to the time
// of the build
//#define SET_CLOCK
#include <Arduino.h>
#include <SPIFFS.h>
#include <SD.h>
#include <m5core2_power.hpp>
#include <m5core2_audio.hpp>
#include <mpu6886.hpp>
#include <bm8563.hpp>
#include <ft6336.hpp>
#include <tft_io.hpp>
#include <ili9341.hpp>
#include <gfx.hpp>
// font for example
// not necessary
#include "Ubuntu.hpp"
using namespace arduino;
using namespace gfx;

// pin assignments
constexpr static const uint8_t spi_host = VSPI;
constexpr static const int8_t lcd_pin_bl = -1;
constexpr static const int8_t lcd_pin_dc = 15;
constexpr static const int8_t lcd_pin_rst = -1;
constexpr static const int8_t lcd_pin_cs = 5;
constexpr static const int8_t lcd_pin_touch_int = 39;
constexpr static const int8_t sd_pin_cs = 4;
constexpr static const int8_t led_pin = 15;
constexpr static const int8_t spi_pin_mosi = 23;
constexpr static const int8_t spi_pin_clk = 18;
constexpr static const int8_t spi_pin_miso = 38;

using bus_t = tft_spi_ex<spi_host, 
                        lcd_pin_cs, 
                        spi_pin_mosi, 
                        -1, 
                        spi_pin_clk, 
                        SPI_MODE0,
                        true, 
                        320 * 240 * 2 + 8, 2>;

using lcd_t = ili9342c<lcd_pin_dc, 
                      lcd_pin_rst, 
                      lcd_pin_bl, 
                      bus_t, 
                      1, 
                      true, 
                      400, 
                      200>;

// lcd colors
using color_t = color<typename lcd_t::pixel_type>;

// the LCD
lcd_t lcd;

// the power module
m5core2_power power;

// the audio module
m5core2_audio sound;

// the real time clock
bm8563 rtc(i2c_container<1>::instance());

// the touch panel
ft6336<280,320,lcd_pin_touch_int> touch(i2c_container<1>::instance());

// the gyro
mpu6886 gyro(i2c_container<1>::instance());

// declare an alias for the I2C port
// in the Grove form factor on the 
// side of the device
TwoWire& grove = i2c_container<0>::instance();

// initialize M5 Stack Core2 peripherals/features
void initialize_m5stack_core2() {
    Serial.begin(115200);
    i2c_container<0>::instance().begin(32, 33);
    i2c_container<1>::instance().begin(21, 22);
    power.initialize();
    rtc.initialize();
    spi_container<spi_host>::instance().begin(spi_pin_clk,spi_pin_miso,spi_pin_mosi,-1);
    SD.begin(4,spi_container<spi_host>::instance());
    lcd.initialize();
    SPIFFS.begin(false);
    lcd.fill(lcd.bounds(),color_t::purple);
    rect16 rect(0,0,64,64);
    rect.center_inplace(lcd.bounds());
    lcd.fill(rect,color_t::white);
    lcd.fill(rect.inflate(-8,-8),color_t::purple);
    gyro.initialize();
    touch.rotation(1);
    touch.interrupt_enabled(false);
    if(!sound.initialize()) {
        Serial.println("Sound initialization failed");
        while(true);
    }
#ifdef SET_CLOCK
    tm build_tm;
    rtc.build(&build_tm);
    rtc.set(build_tm);
    Serial.print("Clock set to ");
    Serial.println(asctime(&build_tm));
#endif
}

void setup() {
    initialize_m5stack_core2();
    
    // your code here

    
    // example - go ahead and delete
    lcd.fill(lcd.bounds(),color_t::black);
    const char* m5_text = "M5Stack";
    constexpr static const uint16_t text_height = 80;
    srect16 text_rect;
    open_text_info text_draw_info;
    const open_font &text_font = Ubuntu;
    
    text_draw_info.text = m5_text;
    text_draw_info.font = &text_font;
    text_draw_info.scale = text_font.scale(text_height);
    text_draw_info.transparent_background = false;
    text_rect = text_font.measure_text(ssize16::max(),
                                    spoint16::zero(),
                                    m5_text,
                                    text_draw_info.scale)
                                        .bounds()
                                            .center((srect16)lcd.bounds())
                                                .offset(0,-text_height/2);
    draw::text(lcd,text_rect,text_draw_info,color_t::gray);
    draw::line(lcd,
            srect16(text_rect.x1,text_rect.y1,text_rect.x1,text_rect.y2)
                .offset(80,0),
            color_t::white);
    const char* core2_text = "Core2";
    text_draw_info.text = core2_text;
    text_rect = text_font.measure_text(
                                    ssize16::max(),
                                    spoint16::zero(),
                                    core2_text,text_draw_info.scale)
                                        .bounds()
                                            .center((srect16)lcd.bounds())
                                                .offset(0,text_height/2);
    draw::text(lcd,text_rect,text_draw_info,color_t::blue);
    tm t;
    
    Serial.println("Booted");

    tm current_tm;
    rtc.now(&current_tm);
    Serial.print("Time is reported as ");
    Serial.println(asctime(&current_tm));
    
    // each of the following (play/record) works by itself but they 
    // crash if you try to use both
    sound.sinw(2000,.05);
    delay(50);
    sound.sinw(1000,.05);
    delay(50); 
    sound.sinw(500,.05);
    delay(50); 
    
    sound.stop();
    /*
    sound.start_recording([](const uint16_t* data, size_t size, void* state) {
        Serial.println(size);
    },nullptr);
    delay(2000);
    sound.stop_recording();
    */

   

}
void loop() {
    touch.update();
    if(touch.pressed()) {
        uint16_t x,y;
        if(touch.xy(&x,&y)) {
            if(y<240) {
                draw::filled_ellipse(lcd,srect16(spoint16(x,y),6),color_t::purple);
            } else {
                Serial.printf("Bottom: x=%03d, y=%03d\n",(int)x,(int)y);
            }
        }
    }
    taskYIELD();
}