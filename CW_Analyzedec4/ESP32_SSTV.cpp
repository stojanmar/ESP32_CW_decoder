#include <Arduino.h>
#include "ESP32_SSTV.h"

extern int16_t getNextSSTVSample();
extern String Modesstv;

c_sstv_decoder_ESP32::c_sstv_decoder_ESP32(
    float fs,
    //Arduino_Canvas* gfx
    Arduino_GFX* gfx
) : c_sstv_decoder(fs)
{
    display = gfx;
}

void c_sstv_decoder_ESP32::start()
{
    // Nothing yet
}

void c_sstv_decoder_ESP32::stop()
{
    // Nothing yet
}

int16_t c_sstv_decoder_ESP32::get_audio_sample()
{
    /*static uint32_t count = 0;

    count++;

    if ((count % 1000) == 0)
        Serial.printf("audio count = %lu\n", count);*/

    return getNextSSTVSample();
}

/*void c_sstv_decoder_ESP32::image_write_line(
    uint16_t line_rgb565[],
    uint16_t y,
    uint16_t width,
    uint16_t height,
    const char* mode_string)
{
    static uint32_t lineCount = 0;

    lineCount++;

    Serial.printf(
        "SSTV LINE %lu: y=%u width=%u height=%u mode=%s\n",
        lineCount,
        y,
        width,
        height,
        mode_string
    );
}*/
void c_sstv_decoder_ESP32::image_write_line(
    uint16_t line_rgb565[],
    uint16_t y,
    uint16_t width,
    uint16_t height,
    const char* mode_string)
{
    const uint16_t IMAGE_Y = 12;
    // Start of a new image
    if (y == 0) {
        display->fillRect(0, IMAGE_Y, 320, 260, RGB565_BLACK);
    }
    // Don't draw outside the display
    if ((IMAGE_Y + y) < 272) {
        display->draw16bitRGBBitmap(
            0,                  // x
            IMAGE_Y + y,        // y
            line_rgb565,        // decoded RGB565 line
            width,              // normally 320 for Scottie S2
            1                   // one line
        );
    }

    

    // Refresh every 7 or 15 lines
    if ((y & 3) == 3) {
    //update progress
    display->fillRect(0, 1, 320, 20, RGB565_BLACK);
    char buffer[21];
    snprintf(buffer, 20, "%10s: %ux%u", mode_string, width, y+1);
    //display->drawString(0, display_height+10, font_8x5, buffer, COLOUR_WHITE, COLOUR_BLACK);   
    display->setCursor(10, 4);
    display->print(String(buffer)); // + "," + String(touchY));
    //Serial.println(buffer);
        //Modesstv = mode_string;
    display->flush();
    }

    // Optional status information
    // ...
}

void c_sstv_decoder_ESP32::scope(uint16_t mag, int16_t freq)
{
    // Nothing yet.
    //
    // Later this can feed your existing
    // SSTV spectrum/waterfall display.
    const uint16_t scope_x = 325;
    const uint16_t scope_y = 250;
    const uint16_t scope_width = 150;

    static uint8_t row=0;
    static uint16_t count=0;
    static uint32_t spectrum[150];
    static uint32_t signal_strength = 0;

    const int8_t f=(freq-1000)*150/1500;
    const uint8_t Hz_1200 = (1200-1000)*scope_width/1500;
    const uint8_t Hz_1500 = (1500-1000)*scope_width/1500;
    const uint8_t Hz_2300 = (2300-1000)*scope_width/1500;
   
    /*if (freq < 2450 && f>0 && f<scope_width) {
      spectrum[f] = (spectrum[f] * 15 + mag)/16;
    }*/
    signal_strength = (signal_strength * 15 + mag)/16;
    if (count>200 ) {
      display->drawRect(scope_x-1, scope_y-6, scope_width+3, 7, RGB565_WHITE);
      uint16_t waterfall[scope_width];
      /*for (int i=0;i<150;i++) {
        float scaled_dB = 2*20*log10(spectrum[i]);
        scaled_dB = std::max(std::min(scaled_dB, 255.0f), 0.0f);
        waterfall[i]=display->color565(0, scaled_dB, scaled_dB);
      }*/
      /*waterfall[Hz_1200]=RGB565_RED;
      waterfall[Hz_1500]=RGB565_RED;
      waterfall[Hz_2300]=RGB565_RED;*/
      //display->writeHLine(scope_x,scope_y-10+row++,150,waterfall);
      /*display->draw16bitRGBBitmap(
            scope_x,                  // x
            scope_y-10+row++,        // y
            waterfall,        // decoded RGB565 line
            150,              // normally 320 for Scottie S2
            1                   // one line
        );*/
      
      /*for (int i=0;i<scope_width;i++) {
        spectrum[i]=0;
      }*/

      if (row>7) row=0;   
      count=0;

      // Draw signal bar
      float scaled_dB = 2*20*log10(signal_strength);
      scaled_dB = std::max(std::min(scaled_dB, 149.0f), 0.0f);
      display->fillRect(scope_x, scope_y-4, scaled_dB, 4, RGB565_CYAN);
      display->fillRect(scope_x+scaled_dB, scope_y-4, 150-scaled_dB, 4, RGB565_BLACK);

    }
    count++;
}
/*void c_sstv_decoder_ESP32::scope(uint16_t mag, int16_t freq)
{
    static uint32_t count = 0;
    static uint32_t bins[16] = {0};

    int bin;

    if (freq < 1100)
        bin = 0;
    else if (freq >= 2500)
        bin = 15;
    else
        bin = 1 + (freq - 1100) / 100;

    bins[bin]++;
    count++;

    if (count >= 15000)
    {
        Serial.println("---- SSTV frequency distribution ----");

        Serial.printf("<1100     : %lu\n", bins[0]);

        for (int i = 1; i <= 14; i++)
        {
            int f1 = 1100 + (i - 1) * 100;
            int f2 = f1 + 99;

            Serial.printf(
                "%4d-%4d : %lu\n",
                f1, f2, bins[i]
            );
        }

        Serial.printf(">=2500     : %lu\n", bins[15]);

        for (int i = 0; i < 16; i++)
            bins[i] = 0;

        count = 0;
    }
}*/