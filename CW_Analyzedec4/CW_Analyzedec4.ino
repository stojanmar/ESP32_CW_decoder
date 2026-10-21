/* CW analyzer + decoder on ESP32-S3 display Guition 4,3" first experimental
* Prepared by Stojan Markic - S52UV , Licence Apache 2.0
* Ver02 optimirano za nemotenje nonblocking funkcije v sstv library
* Ver03 še več diagnostike zdaj na 1,9 inch
* Ver03a izločena diagnostika za UNSTABLE/STABLE
* Ver03b Finalize at 2.5 dits rather than 3.0. prej zaključi črko in besedo
* Ver03c Dodano še zgodnejše zaključevanje besed na 6 dits namesto standard 7 dits
* Prepared by Stojan Markic, S52UV , Sept 2026
* CW latency test: removed STABLE diagnostic; ON/OFF confirmation 8/12 ms.
* Ver03d Dodani markerji za tunning to 800Hz
* Ver03e Pohitren algoritem za adaptive WPM
* Ver04 Lower trashold and aditional filter during sampling
*/
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sstv_decoder.h"
#include "half_band_filter2.h"
#include "cordic.h"
#include "ESP32_SSTV.h"

#include <Arduino_GFX_Library.h>
//#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>
#include "driver/adc.h"
#include "hal/adc_ll.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
//#include "src/tamc_gt911/TAMC_GT911.h" //https://github.com/TAMCTec/gt911-arduino

/*#define TOUCH_SDA  (gpio_num_t)8
#define TOUCH_SCL  (gpio_num_t)4
#define TOUCH_INT  (gpio_num_t)3
#define TOUCH_RST  (gpio_num_t)38
#define TOUCH_WIDTH  480
#define TOUCH_HEIGHT 272*/

//LCD  JC4827W543C
/*#define LCD_BL  (gpio_num_t)1
#define LCD_CS  (gpio_num_t)45
#define LCD_SCK (gpio_num_t)47
#define LCD_D0  (gpio_num_t)21
#define LCD_D1  (gpio_num_t)48
#define LCD_D2  (gpio_num_t)40
#define LCD_D3  (gpio_num_t)39
#define LCD_RST (gpio_num_t)-1*/

#define NUM_SAMPLES 2048 //1024           // Number of samples per capture

// ADC1 Channel 0 is GPIO1 on ESP32-S3
#define ADC_CHANNEL ADC1_CHANNEL_6  // reads pin34
#define ADC_CHANNEL2 ADC1_CHANNEL_6  // reads pin34
#define ADC_ATTEN   ADC_ATTEN_DB_11 // 11dB attenuation for ~0-3.1V range
// Define the ADC midpoint
#define ADC_MIDPOINT 1950 //512


// Forward declaration: the Arduino IDE auto-generates function prototypes and inserts
// them near the top of the file (before the real "struct MNode" definition further
// down). Without this forward declaration, that auto-generated prototype for
// newNode() (which returns MNode*) would reference an as-yet-unknown type and fail
// to compile.
struct MNode;

// CYD 1.9" ST7789
// Physical display is used in landscape:
//   320 x 170
#define GFX_BL 21

Arduino_DataBus *bus =
    new Arduino_HWSPI(
        2,                  // DC
        15,                 // CS
        14,                 // SCK
        13,                 // MOSI
        GFX_NOT_DEFINED     // MISO
    );

Arduino_GFX *gfx =
    new Arduino_ST7789(
        bus,
        GFX_NOT_DEFINED,
        1,                  // rotation
        true,               // IPS
        170,                // width in constructor
        320,                // height in constructor
        0,                  // col offset 1
        0,                  // row offset 1
        35,
        0
    );

LGFX_Sprite sprite;
LGFX_Sprite spr2;

#define DISPLAY_W 320
#define DISPLAY_H 170

const int16_t STATUS_BAR_H  = 22;
const size_t  MAX_DISPLAY_CHARS = 104; //130;   // rolling text window sized to fit the screen
#define COLOR_NAVY 0x0010   // RGB565 dark navy blue -- ST77XX_NAVY isn't defined by the library

String decodedText = "";


// for touch
bool tp_control = true;
bool tp_band = false;
uint16_t in_left = 999; 
bool l_release = false; 
bool t_release = false; 
bool l_press = false; 
bool t_press = false; 
uint8_t lkey = 1;

/*TAMC_GT911 tp = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);
int tp_x,tp_y,tp_size;
// Touch coordinates
int x = 0;
int y = 0;
int z = 0;*/

int16_t getNextSSTVSample();  //prototype declaration
bool imageInProgress = false;
bool imageComplete = false;
bool stopped = false;

//c_sstv_decoder_ESP32 decoder(15000.0f, gfx);
c_sstv_decoder_ESP32 decoder(14925.0f, gfx);

//uint16_t touchX, touchY;

int oldsamples1[NUM_SAMPLES];
//int oldsamples2[NUM_SAMPLES];
volatile int index1 = 0;
unsigned long currentmicro;
unsigned long cur_ms   = 0;
unsigned int difmicro;
bool redraw = false;
volatile bool filled = false;
int picturenum = 0;

int fps;
int dummy;
String Modesstv = "Not received";

//double buffer alternating
#define BUF_SIZE 2048 //480
#define SSTV_BLOCK_SIZE 1024
volatile int16_t bufA[BUF_SIZE];
volatile int16_t bufB[BUF_SIZE];
// Processed 15 kHz samples
int16_t sstvSamples[SSTV_BLOCK_SIZE];
volatile uint32_t bufferOverrunA = 0;
volatile uint32_t bufferOverrunB = 0;

volatile bool useA = true;
volatile uint16_t idx = 0;
volatile bool readyA = false;
volatile bool readyB = false;

uint32_t lastUIUpdateMs = 0;

// -------- FAST ADC READ --------
int IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value;
    adc_ll_rtc_enable_channel(ADC_NUM_1, channel);
    adc_ll_rtc_start_convert(ADC_NUM_1, channel);
    
    while (adc_ll_rtc_convert_is_done(ADC_NUM_1) == 0);
    adc_value = adc_ll_rtc_get_convert_value(ADC_NUM_1);
    
    return adc_value;
   }

 
hw_timer_t *Timer1_Cfg = NULL;
  
void IRAM_ATTR Timer1_ISR() {
  //currentmicro = micros();
    //digitalWrite(interruptPin, HIGH);
    // Toggle GPIO pin
    //digitalWrite(interruptPin, !digitalRead(interruptPin));
     if (useA) {
        bufA[idx] = local_adc1_read(ADC_CHANNEL); // * 4;  //adc1_get_raw(ADC_CHANNEL); //analogRead(6);

    } else {
        bufB[idx] = local_adc1_read(ADC_CHANNEL); // * 4;  //adc1_get_raw(ADC_CHANNEL); //analogRead(6);
        
    }
      
    idx++;

    if (idx >= BUF_SIZE) {
        if (useA) readyA = true;
        else      readyB = true;
        useA = !useA;
        idx = 0;
        //filled = true;
    }
    //difmicro = micros() - currentmicro;
    //digitalWrite(interruptPin, LOW);
}

 // Return one 15 kHz low-pass-filtered audio sample, while sampling is at 30kHz
int16_t getNextSSTVSample()
{
    static uint16_t sample_number = SSTV_BLOCK_SIZE;

    // One-pole low-pass filter:
    // Fs = 15000 Hz, cutoff approximately 1000 Hz.
    // alpha = 11200 / 32768 = 0.342
    static int32_t lp = 0;
    const int32_t FILTER_ALPHA_Q15 = 11200;

    if (sample_number >= SSTV_BLOCK_SIZE)
    {
        volatile int16_t *rawSamples = nullptr;

        // Wait until a raw buffer is ready.
        while (!readyA && !readyB)
        {
            delay(0);
        }

        if (readyA)
        {
            readyA = false;
            rawSamples = bufA;
        }
        else
        {
            readyB = false;
            rawSamples = bufB;
        }

        // Convert 2048 raw samples into 1024 processed samples.
        static int32_t dc = 0;

        for (uint16_t i = 0; i < SSTV_BLOCK_SIZE; i++)
        {
            int32_t sample =
                (int32_t)rawSamples[i * 2] +
                (int32_t)rawSamples[i * 2 + 1];

            dc += (sample - dc) / 2;
            sample -= dc;
            sample <<= 2;

            if (sample > 32767) sample = 32767;
            else if (sample < -32768) sample = -32768;

            // Low-pass filter: attenuate frequencies above ~1 kHz.
            lp += ((sample - lp) * FILTER_ALPHA_Q15) >> 15;

            sstvSamples[i] = (int16_t)lp;
        }

        sample_number = 0;
    }

    return sstvSamples[sample_number++];
}

// ---------------- CW ANALYZER UI ----------------
enum CWPage { CW_SETUP_PAGE, CW_ANALYSIS_PAGE };
CWPage cwPage = CW_ANALYSIS_PAGE; //CW_SETUP_PAGE;

int cwCenterHz = 800;       // Default pitch
int cwBandwidthHz = 100; //60;     // +/- bandwidth
const int CW_CENTER_MIN = 300;
const int CW_CENTER_MAX = 1500;
const int CW_BW_MIN = 20;
const int CW_BW_MAX = 200;

uint16_t cwMagnitude = 0;
int16_t cwFrequencyHz = 0;
bool cwFrequencyValid = false;
bool cwTonePresent = false;

bool cwTouchWasDown = false;
int cwTouchDownX = 0;
int cwTouchDownY = 0;
uint32_t cwLastDisplayMs = 0;
uint32_t cwLastToneChangeMs = 0;
uint32_t cwLastTouchActionMs = 0;
uint32_t cwIgnoreTouchUntilMs = 0;
bool cwCandidateTone = false;
uint32_t cwCandidateSinceSample = 0;
uint32_t cwOffSinceSample = 0;

// The CW DSP consumes one 15 kHz audio sample per read. Timing transitions in
// this sample clock, not millis(), so UI redraw delays cannot stretch marks.
static const uint32_t CW_AUDIO_FS = 14925;
uint32_t cwAudioSampleClock = 0;
uint32_t cwAudioMsClock = 0;
uint32_t cwAudioMsRemainder = 0;
uint32_t cwLastTouchPollMs = 0;

// ---------------- CW MORSE DECODER ----------------
// Timing follows detected tone edges, so the existing SSTV-derived DSP and
// tone detector remain unchanged. Dit estimate starts at 100 ms (12 WPM).
uint32_t cwMarkStartSample = 0;
uint32_t cwGapStartSample = 0;
bool cwTimingToneState = false;
bool cwGapActive = false;
bool cwLetterFinalized = false;
bool cwWordSpaceAdded = false;
float cwDitMs = 100.0f;
// Reacquire a changed speed from repeated short intra-character gaps.
float cwGapDitCandidateMs = 0.0f;
uint8_t cwGapDitCandidateCount = 0;
char cwMorseCode[10] = "";
char cwDecodedText[241] = "";
uint16_t cwDecodedCount = 0;

struct MorseEntry { const char *code; char ch; };
const MorseEntry cwMorseTable[] = {
  {".-",'A'}, {"-...",'B'}, {"-.-.",'C'}, {"-..",'D'}, {".",'E'},
  {"..-.",'F'}, {"--.",'G'}, {"....",'H'}, {"..",'I'}, {".---",'J'},
  {"-.-",'K'}, {".-..",'L'}, {"--",'M'}, {"-.",'N'}, {"---",'O'},
  {".--.",'P'}, {"--.-",'Q'}, {".-.",'R'}, {"...",'S'}, {"-",'T'},
  {"..-",'U'}, {"...-",'V'}, {".--",'W'}, {"-..-",'X'}, {"-.--",'Y'},
  {"--..",'Z'}, {"-----",'0'}, {".----",'1'}, {"..---",'2'},
  {"...--",'3'}, {"....-",'4'}, {".....",'5'}, {"-....",'6'},
  {"--...",'7'}, {"---..",'8'}, {"----.",'9'}, {".-.-.-",'.'},
  {"--..--",','}, {"..--..",'?'}, {"-.-.--",'!'}, {".-..-.",'"'},
  {"-.--.",'('}, {"-.--.-",')'}, {".-.-.",'+'}, {"-....-",'-'},
  {".-.-",'+'}, {"...-..-",'$'}, {"-..-.",'/'}, {"-.--.",'('},
  {"---...",':'}, {"-.-.-.",';'}, {"-...-",'='}, {".-.-.",'+'},
  {"..--.-",'_'}, {".-...",'&'}, {".--.-.",'@'}
};

void clearCWDecoder()
{
  cwMarkStartSample = 0;
  cwGapStartSample = 0;
  cwTimingToneState = false;
  cwGapActive = false;
  cwLetterFinalized = false;
  cwWordSpaceAdded = false;
  cwDitMs = 100.0f;
  cwGapDitCandidateMs = 0.0f;
  cwGapDitCandidateCount = 0;
  cwMorseCode[0] = '\0';
  cwDecodedText[0] = '\0';
  cwDecodedCount = 0;
}

void appendCWText(char ch)
{
  size_t len = strlen(cwDecodedText);
  if (len >= sizeof(cwDecodedText) - 2) {
    // Keep the newest text visible instead of allowing the buffer to grow.
    memmove(cwDecodedText, cwDecodedText + 80, len - 79);
    cwDecodedText[len - 80] = '\0';
    len = strlen(cwDecodedText);
  }
  cwDecodedText[len] = ch;
  cwDecodedText[len + 1] = '\0';
  cwDecodedCount++;
}

char decodeCWMorse(const char *code)
{
  if (!code || !code[0]) return '\0';
  for (size_t i = 0; i < sizeof(cwMorseTable) / sizeof(cwMorseTable[0]); ++i) {
    if (strcmp(code, cwMorseTable[i].code) == 0) return cwMorseTable[i].ch;
  }
  return '#';
}

void finalizeCWLetter()
{
  if (cwMorseCode[0] == '\0') return;
  appendCWText(decodeCWMorse(cwMorseCode));
  cwMorseCode[0] = '\0';
  cwLetterFinalized = true;
}

void addCWMorseElement(uint32_t markSamples)
{
  // Convert sample count to milliseconds only after measuring the edge-to-edge
  // interval in the audio stream's own clock.
  uint32_t markMs = (uint32_t)(((uint64_t)markSamples * 1000ULL) / CW_AUDIO_FS);
  if (markMs < 20 || markMs > 2500) return;

  bool isDah = markMs >= cwDitMs * 2.0f;
  float measuredDit = isDah ? (markMs / 3.0f) : (float)markMs;
  if (measuredDit >= 25.0f && measuredDit <= 500.0f) {
    // Normal tracking is faster than before, and accepts a wider speed change.
    // Large changes are primarily reacquired from repeated short gaps below.
    float markUnits = markMs / cwDitMs;
    float ratio = measuredDit / cwDitMs;
    bool plausibleDot = !isDah && markUnits >= 0.45f && markUnits <= 1.65f;
    bool plausibleDah = isDah && markUnits >= 2.1f && markUnits <= 4.2f;
    if ((plausibleDot || plausibleDah) && ratio >= 0.60f && ratio <= 1.60f)
      cwDitMs = cwDitMs * 0.80f + measuredDit * 0.20f;
    if (cwDitMs < 25.0f) cwDitMs = 25.0f;
    if (cwDitMs > 500.0f) cwDitMs = 500.0f;
  }

  size_t len = strlen(cwMorseCode);
  if (len < sizeof(cwMorseCode) - 2) {
    cwMorseCode[len] = isDah ? '-' : '.';
    cwMorseCode[len + 1] = '\0';
  }
  cwLetterFinalized = false;
  cwWordSpaceAdded = false;
}

void processCWToneEdges()
{
  const uint32_t ON_CONFIRM_SAMPLES = (CW_AUDIO_FS * 8UL) / 1000UL;
  const uint32_t OFF_CONFIRM_SAMPLES = (CW_AUDIO_FS * 12UL) / 1000UL;
  uint32_t nowSample = cwAudioSampleClock;

  if (cwTonePresent && !cwTimingToneState) {
    // Backdate the rising edge to compensate the 8 ms detector confirmation.
    uint32_t startSample = (nowSample > ON_CONFIRM_SAMPLES) ? nowSample - ON_CONFIRM_SAMPLES : 0;
    if (cwGapActive) {
      uint32_t gapSamples = startSample - cwGapStartSample;
      // Classify this gap using the estimate that existed before this sample.
      uint32_t ditSamples = (uint32_t)(cwDitMs * (CW_AUDIO_FS / 1000.0f));
      float oldDitMs = cwDitMs;
      float gapMs = (float)gapSamples * 1000.0f / CW_AUDIO_FS;

      // A Morse intra-character gap is one dit. If the generator speed has
      // changed substantially, repeated short gaps can reacquire the new dit
      // length even when mark classification is temporarily ambiguous.
      // Require two similar observations to avoid reacting to one odd gap.
      if (gapSamples < (ditSamples * 5UL) / 2UL && gapMs >= 20.0f && gapMs <= 500.0f) {
        float gapRatio = gapMs / oldDitMs;
        if (gapRatio >= 0.40f && gapRatio <= 2.40f) {
          if (cwGapDitCandidateCount > 0 && cwGapDitCandidateMs > 0.0f &&
              gapMs >= cwGapDitCandidateMs * 0.75f &&
              gapMs <= cwGapDitCandidateMs * 1.25f) {
            if (cwGapDitCandidateCount < 3) ++cwGapDitCandidateCount;
          } else {
            cwGapDitCandidateMs = gapMs;
            cwGapDitCandidateCount = 1;
          }
          if (cwGapDitCandidateCount >= 2 && fabsf(gapMs - oldDitMs) > oldDitMs * 0.25f) {
            cwDitMs = oldDitMs * 0.65f + gapMs * 0.35f;
            if (cwDitMs < 25.0f) cwDitMs = 25.0f;
            if (cwDitMs > 500.0f) cwDitMs = 500.0f;
            cwGapDitCandidateCount = 0;
            cwGapDitCandidateMs = 0.0f;
          }
        }
      } else {
        cwGapDitCandidateCount = 0;
        cwGapDitCandidateMs = 0.0f;
      }

      if (gapSamples >= (ditSamples * 6UL)) {
        finalizeCWLetter();
        if (!cwWordSpaceAdded && cwDecodedText[0] && cwDecodedText[strlen(cwDecodedText) - 1] != ' ')
          appendCWText(' ');
        cwWordSpaceAdded = true;
      } else if (gapSamples >= (ditSamples * 5UL) / 2UL) {
        // Finalize at 2.5 dits rather than 3.0. This gives the decoder
        // margin before the next letter starts at higher CW speeds.
        finalizeCWLetter();
      }
    }
    cwGapActive = false;
    cwMarkStartSample = startSample;
    cwTimingToneState = true;
  } else if (!cwTonePresent && cwTimingToneState) {
    uint32_t endSample = (nowSample > OFF_CONFIRM_SAMPLES) ? nowSample - OFF_CONFIRM_SAMPLES : 0;
    uint32_t markSamples = endSample - cwMarkStartSample;
    addCWMorseElement(markSamples);
    cwGapStartSample = endSample;
    cwGapActive = true;
    cwTimingToneState = false;
    cwLetterFinalized = false;
    cwWordSpaceAdded = false;
  }
}

void updateCWMorseSpacing()
{
  if (!cwGapActive) return;
  uint32_t gapSamples = cwAudioSampleClock - cwGapStartSample;
  uint32_t ditSamples = (uint32_t)(cwDitMs * (CW_AUDIO_FS / 1000.0f));
  if (!cwLetterFinalized && gapSamples >= (ditSamples * 5UL) / 2UL) {
    // Slightly early inter-letter decision (2.5 dit units) to avoid
    // merging the next letter at higher speeds such as 25-30 WPM.
    finalizeCWLetter();
  }
  if (!cwWordSpaceAdded && gapSamples >= (ditSamples * 6UL)) {
    // Word gap is seven dit units. Insert one space after the letter is done.
    finalizeCWLetter();
    if (cwDecodedText[0] && cwDecodedText[strlen(cwDecodedText) - 1] != ' ')
      appendCWText(' ');
    cwWordSpaceAdded = true;
  }
}

void advanceCWAudioClock()
{
  ++cwAudioSampleClock;
  cwAudioMsRemainder += 1000;
  if (cwAudioMsRemainder >= CW_AUDIO_FS) {
    cwAudioMsRemainder -= CW_AUDIO_FS;
    ++cwAudioMsClock;
  }
}

int cwEstimatedWPM()
{
  if (cwDitMs < 1.0f) return 0;
  return (int)(1200.0f / cwDitMs + 0.5f);
}

void drawCWSetup()
{
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextWrap(false);

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_CYAN);
  gfx->setCursor(20, 12);
  gfx->print("CW RECEIVER SETUP");

  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(28, 55);
  gfx->print("Center tone:");

  gfx->fillRoundRect(35, 82, 65, 44, 6, RGB565_DARKGREY);
  gfx->fillRoundRect(380, 82, 65, 44, 6, RGB565_DARKGREY);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(55, 94);
  gfx->print("-");
  gfx->setCursor(400, 94);
  gfx->print("+");
  gfx->setTextColor(RGB565_YELLOW);
  gfx->setCursor(155, 94);
  gfx->print(cwCenterHz);
  gfx->print(" Hz");

  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(28, 145);
  gfx->print("Bandwidth:");

  gfx->fillRoundRect(35, 169, 65, 44, 6, RGB565_DARKGREY);
  gfx->fillRoundRect(380, 169, 65, 44, 6, RGB565_DARKGREY);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(55, 181);
  gfx->print("-");
  gfx->setCursor(400, 181);
  gfx->print("+");
  gfx->setTextColor(RGB565_YELLOW);
  gfx->setCursor(160, 181);
  gfx->print("+/- ");
  gfx->print(cwBandwidthHz);
  gfx->print(" Hz");

  gfx->setTextColor(RGB565_CYAN);
  gfx->setCursor(28, 225);
  gfx->print("Range ");
  gfx->print(cwCenterHz - cwBandwidthHz);
  gfx->print("-");
  gfx->print(cwCenterHz + cwBandwidthHz);
  gfx->print(" Hz");

  gfx->fillRoundRect(315, 224, 145, 40, 6, RGB565_GREEN);
  gfx->setTextColor(RGB565_BLACK);
  gfx->setCursor(329, 236);
  gfx->print("START CW");
  //gfx->flush();
}

void drawCWAnalysis()
{
  sprite.fillRect(0,0,320,147,RGB565_BLACK);
  sprite.setTextWrap(false);
  sprite.setTextSize(2);

  sprite.setCursor(8, 8);
  //gfx->print("Freq ");
  //gfx->setTextColor(cwFrequencyValid ? RGB565_YELLOW : RGB565_DARKGREY);
  //if (cwFrequencyValid) { gfx->print(cwFrequencyHz); gfx->print(" Hz"); }
  //else gfx->print("--");
  sprite.setTextColor(RGB565_WHITE);
  sprite.print("Mag "); sprite.setTextColor(RGB565_YELLOW); sprite.print(cwMagnitude);

  /*sprite.setTextColor(RGB565_WHITE);
  sprite.setCursor(8, 28);
  sprite.print("Tone ");
  sprite.setTextColor(cwTonePresent ? RGB565_GREEN : RGB565_RED);
  sprite.print(cwTonePresent ? "ON " : "OFF");*/
  //gfx->setTextColor(RGB565_WHITE);
  //gfx->print("  WPM "); gfx->setTextColor(RGB565_CYAN); gfx->print(cwEstimatedWPM());

  sprite.setTextColor(RGB565_WHITE);
  sprite.setCursor(8, 28);
  sprite.print("Morse: ");
  sprite.setTextColor(RGB565_YELLOW);
  sprite.print(cwMorseCode[0] ? cwMorseCode : "-");

  // Show the newest decoded text in three compact lines. The text buffer is
  // deliberately retained across redraws and is cleared only by START CW.
  const int maxCharsPerLine = 25;
  size_t len = strlen(cwDecodedText);
  size_t start = (len > 3 * maxCharsPerLine) ? len - 3 * maxCharsPerLine : 0;
  //sprite.setTextSize(2);
  for (int line = 0; line < 3; ++line) {
    sprite.setCursor(8, 50 + line * 20);
    size_t pos = start + line * maxCharsPerLine;
    for (int j = 0; j < maxCharsPerLine && pos + j < len; ++j)
      sprite.print(cwDecodedText[pos + j]);
  }

  // Bottom plot area
  const int plotTop = 117;
  const int plotHeight = 30;

  // Fixed 800 Hz reference marker in the bottom 40 pixels
  // Fixed 800 Hz reference line
  const int refX = 160;
  sprite.drawFastVLine(refX, plotTop, plotHeight, RGB565_RED);

  // ---- Filtered dynamic tone marker ----
  // Frequency range: 600 Hz at x=0, 1000 Hz at x=319
  // Magnitude 800 or higher reaches full height.
  static float filteredX = 160.0f;
  static float filteredHeight = 0.0f;

  if (cwFrequencyValid &&
      cwFrequencyHz >= 600 && cwFrequencyHz <= 1000) {

    // Map frequency to horizontal position
    float targetX = (cwFrequencyHz - 600) * (319.0f / 400.0f);

    // Map magnitude to height, clamped to 0..40 pixels
    float targetHeight = cwMagnitude * (30.0f / 800.0f);
    if (targetHeight > 30.0f) targetHeight = 30.0f;
    if (targetHeight < 0.0f) targetHeight = 0.0f;

    // Exponential smoothing: lower values = more smoothing
    const float X_ALPHA = 0.15f;
    const float AMP_ALPHA = 0.15f;

    filteredX += X_ALPHA * (targetX - filteredX);
    filteredHeight += AMP_ALPHA * (targetHeight - filteredHeight);

    int markerX = (int)(filteredX + 0.5f);
    int markerH = (int)(filteredHeight + 0.5f);

    if (markerH > plotHeight) markerH = plotHeight;
    if (markerH < 1) markerH = 1;

    // Draw upward from the bottom; 3 pixels wide, green
    sprite.drawFastVLine(markerX - 1,
                         plotTop + plotHeight - markerH,
                         markerH, RGB565_GREEN);
    sprite.drawFastVLine(markerX,
                         plotTop + plotHeight - markerH,
                         markerH, RGB565_GREEN);
    sprite.drawFastVLine(markerX + 1,
                         plotTop + plotHeight - markerH,
                         markerH, RGB565_GREEN);
  }

  gfx->draw16bitBeRGBBitmap(0, 23, (uint16_t*)sprite.getBuffer(), 320, 147);  //to je kot pushsprite je Big endian
    //gfx->draw16bitRGBBitmap(0, 0, (uint16_t*)buf, 320, 170);  //to je kot pushsprite
}

void handleCWTouch(int px, int py)
{
  if (cwPage == CW_SETUP_PAGE) {
    // Center frequency +/- 25 Hz
    if (py >= 78 && py <= 130) {
      if (px >= 25 && px <= 110) cwCenterHz -= 25;
      else if (px >= 370 && px <= 455) cwCenterHz += 25;
    }
    // Bandwidth +/- 10 Hz
    else if (py >= 165 && py <= 216) {
      if (px >= 25 && px <= 110) cwBandwidthHz -= 10;
      else if (px >= 370 && px <= 455) cwBandwidthHz += 10;
    }
    // Start
    else if (px >= 305 && px <= 470 && py >= 218 && py <= 270) {
      cwPage = CW_ANALYSIS_PAGE;
      cwIgnoreTouchUntilMs = millis() + 400;
      cwTonePresent = false;
      cwCandidateTone = false;
      cwCandidateSinceSample = 0;
      cwOffSinceSample = 0;
      cwAudioSampleClock = 0;
      cwAudioMsClock = 0;
      cwAudioMsRemainder = 0;
      cwLastDisplayMs = millis();
      clearCWDecoder();
      drawCWAnalysis();
      return;
    }

    cwCenterHz = constrain(cwCenterHz, CW_CENTER_MIN, CW_CENTER_MAX);
    cwBandwidthHz = constrain(cwBandwidthHz, CW_BW_MIN, CW_BW_MAX);
    drawCWSetup();
  } else {
    // Strict hitbox: only the visible SETUP button can leave analysis.
    if (px >= 330 && px <= 460 && py >= 224 && py <= 264) {
      cwPage = CW_SETUP_PAGE;
      cwIgnoreTouchUntilMs = millis() + 400;
      drawCWSetup();
    }
  }
}

/*void pollCWTouch()
{
  tp.read();
  if (tp.isTouched) {
    int px = tp.points[0].x;
    int py = tp.points[0].y;
    if (!cwTouchWasDown) {
      cwTouchWasDown = true;
      cwTouchDownX = px;
      cwTouchDownY = py;
    }
  } else if (cwTouchWasDown) {
    cwTouchWasDown = false;

    // Handle one action per completed touch. Ignore quick repeats/bounce,
    // especially the original START press after the page has changed.
    uint32_t now = millis();
    if ((int32_t)(now - cwIgnoreTouchUntilMs) < 0) return;
    if (now - cwLastTouchActionMs < 250) return;

    cwLastTouchActionMs = now;
    handleCWTouch(cwTouchDownX, cwTouchDownY);
  }
}*/

void updateCWDetector()
{
  // Short confirmation intervals are measured in decoded audio samples (14.925 kHz),
  // not wall-clock time. This remains correct even if a screen refresh blocks.
  const uint16_t MAG_ON = 70; //90; //100;
  const uint16_t MAG_OFF = 60; //70; //80;
  const uint32_t ON_CONFIRM_SAMPLES = (CW_AUDIO_FS * 8UL) / 1000UL;
  const uint32_t OFF_CONFIRM_SAMPLES = (CW_AUDIO_FS * 12UL) / 1000UL;
  uint32_t nowSample = cwAudioSampleClock;

  bool inFrequencyWindow = cwFrequencyValid &&
      cwFrequencyHz >= (cwCenterHz - cwBandwidthHz) &&
      cwFrequencyHz <= (cwCenterHz + cwBandwidthHz);
  bool candidate = inFrequencyWindow && cwMagnitude >= MAG_ON;

  if (!cwTonePresent) {
    cwOffSinceSample = 0;
    if (candidate) {
      if (!cwCandidateTone) {
        cwCandidateTone = true;
        cwCandidateSinceSample = nowSample;
      } else if (nowSample - cwCandidateSinceSample >= ON_CONFIRM_SAMPLES) {
        cwTonePresent = true;
        cwLastToneChangeMs = cwAudioMsClock;
        cwCandidateTone = false;
      }
    } else {
      cwCandidateTone = false;
      cwCandidateSinceSample = 0;
    }
  } else {
    if (!inFrequencyWindow || cwMagnitude < MAG_OFF) {
      if (cwOffSinceSample == 0) cwOffSinceSample = nowSample;
      if (nowSample - cwOffSinceSample >= OFF_CONFIRM_SAMPLES) {
        cwTonePresent = false;
        cwCandidateTone = false;
        cwCandidateSinceSample = 0;
        cwOffSinceSample = 0;
        cwLastToneChangeMs = cwAudioMsClock;
      }
    } else {
      cwOffSinceSample = 0;
    }
  }
}
void redrawText() {
  sprite.fillRect(0, 0, 320, 80, TFT_BLACK);
  sprite.setCursor(0, 10);
  /*sprite.setTextSize(2);
  sprite.setTextColor(TFT_GREEN);
  sprite.setTextWrap(true);*/
  sprite.print(decodedText);
  gfx->draw16bitBeRGBBitmap(0, 22, (uint16_t*)sprite.getBuffer(), 320, 80);  //to je kot pushsprite je Big endian
    //gfx->draw16bitRGBBitmap(0, 0, (uint16_t*)buf, 320, 170);  //to je kot pushsprite

}

void drawStaticUI() {
  gfx->fillScreen(0);
  gfx->fillRect(0, 0, 320, STATUS_BAR_H, COLOR_NAVY);
}

void updateStatusBar() {
  //gfx->fillRect(0, 0, 320 - 16, STATUS_BAR_H, COLOR_NAVY);
  spr2.fillRect(0,0,320 - 16,STATUS_BAR_H,COLOR_NAVY);  // pobriše bar
  spr2.setTextSize(2);
  spr2.setCursor(4, 4);
  /*gfx->setTextSize(1);
  gfx->setTextColor(WHITE);*/
  spr2.setTextColor(TFT_YELLOW,COLOR_NAVY);
  float wpm = cwEstimatedWPM(); //1200.0f / ditLenMs;

  spr2.printf("WPM:%2.0f  Tone:%4dHz", wpm, cwFrequencyHz);

  //gfx->fillCircle(320 - 10, STATUS_BAR_H / 2, 6, toneOn ? GREEN : RED);
  spr2.fillCircle(320 - 10, STATUS_BAR_H / 2, 6, cwTonePresent ? TFT_GREEN : TFT_RED);  //pika za ton
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t*)spr2.getBuffer(), 320, 22);  //to je kot pushsprite je Big endian
    //gfx->draw16bitRGBBitmap(0, 0, (uint16_t*)buf, 320, 170);  //to je kot pushsprite

}

// CORE 0 UI TASK for graphics
void uiTask(void *parameter)
{ 
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
  gfx->begin();
  gfx->fillScreen(BLACK);
  drawStaticUI();

  sprite.setColorDepth(16);
  sprite.createSprite(320,147);
  sprite.setTextSize(2);
  sprite.setTextColor(TFT_GREEN);
  sprite.setTextWrap(true);
  spr2.setColorDepth(16);
  spr2.createSprite(320,22);

    for (;;)
    {
        // Do graphics refresh
    uint32_t nowMs = millis();

  // Turn the character-indicator LED back off once its blink window has elapsed.
  /*if (ledOffTimeMs != 0 && (int32_t)(nowMs - ledOffTimeMs) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOffTimeMs = 0;
  }*/

  if (nowMs - lastUIUpdateMs > 50) {
    lastUIUpdateMs = nowMs;
    updateStatusBar();
    //redrawText();
    drawCWAnalysis(); 
    /*Serial.printf("floor=%.0f peak=%.0f wpm=%.1f tone=%d freq=%dHz\n",
                   noiseFloor, peakLevel, 1200.0f / ditLenMs, toneOn, dominantFreq);*/
  }

        // Give Core 0 a little breathing room
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void setup()
{
  Serial.begin(115200);
  delay(800);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
  delay(10);
  (void)adc1_get_raw(ADC1_CHANNEL_6);

  Timer1_Cfg = timerBegin(0, 40, true);
  timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
  timerAlarmWrite(Timer1_Cfg, 67, true);
  timerAlarmEnable(Timer1_Cfg);

  // START CORE 0 UI TASK for graphics
    xTaskCreatePinnedToCore(
        uiTask,
        "CW_UI",
        4096,
        NULL,
        1,
        NULL,
        0);

  //drawCWSetup();
}

void loop()
{
  
  const uint8_t CW_SAMPLE_BATCH = 32;
  for (uint8_t i = 0; i < CW_SAMPLE_BATCH; ++i) {
    uint16_t mag = 0;
    int16_t freq = 0;
    bool valid = decoder.read_cw_analysis(mag, freq);
    advanceCWAudioClock();

    if (cwPage == CW_ANALYSIS_PAGE) {
      cwMagnitude = mag;
      cwFrequencyValid = valid;
      if (valid) {
        cwFrequencyHz = freq;
      }

      updateCWDetector();
      processCWToneEdges();
      updateCWMorseSpacing();
    }
  }
}
