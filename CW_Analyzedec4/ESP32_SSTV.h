#ifndef ESP32_SSTV_H
#define ESP32_SSTV_H
#include <Arduino_GFX_Library.h>
#include "sstv_decoder.h"

class c_sstv_decoder_ESP32 : public c_sstv_decoder
{

private:
    //Arduino_Canvas* display;
    Arduino_GFX* display;
    int16_t* currentBuffer = nullptr;
    uint16_t currentSample = 0;

public:
    /*c_sstv_decoder_ESP32(float Fs)
        : c_sstv_decoder(Fs)
    {
    }*/
    
    //c_sstv_decoder_ESP32(float fs, Arduino_Canvas* gfx);
    c_sstv_decoder_ESP32(float fs, Arduino_GFX* gfx);

    void start();
    void stop();

protected:

    // Audio input
    int16_t get_audio_sample() override;

    // Decoded image line
    void image_write_line(
        uint16_t line_rgb565[],
        uint16_t y,
        uint16_t width,
        uint16_t height,
        const char* mode_string
    ) override;

    // SSTV frequency/scope information
    void scope(uint16_t mag, int16_t freq) override;
};

#endif