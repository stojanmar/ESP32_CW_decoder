# CW Analyzer Starter for JC4827W543C

This is an experimental CW analysis mode based on the supplied SSTV project.

## Files
- `CW_Analyzer.ino`: replacement sketch based on the uploaded SSTV sketch.
- `sstv_decoder.h/.cpp`: adds `read_cw_analysis()` while retaining the existing SSTV decoder functions.
- Other library files are copied unchanged from the supplied ZIP.

## Installation
1. Back up your working SSTV project.
2. Copy the files from this folder into a separate Arduino sketch folder named `CW_Analyzer`.
3. Keep the same library dependencies and board configuration as the SSTV project.
4. Compile and upload. This has not been compiled on the user's exact Arduino installation, so treat it as a first integration draft.
5. Setup defaults: center 800 Hz, bandwidth +/-60 Hz.
6. Touch the minus/plus buttons to change center in 25 Hz steps and bandwidth in 10 Hz steps. Tap START CW, then SETUP to return.

## Important behavior
- The original ADC timer ISR and double-buffer sampler are retained.
- CW mode uses the same I/Q filter and CORDIC code as SSTV, but does not call SSTV image decoding.
- The frequency output bypasses the SSTV-specific 1000-2500 Hz clamp.
- Magnitude thresholds 100 ON / 80 OFF are provisional based on the reported microphone measurements.
- Frequency stability is only a diagnostic indicator in this version; it does not gate tone detection.
- This version is for analyzing a tone and testing touchscreen settings, not yet a finished Morse decoder.
- It is designed for ESP32 Arduino core 2.x, matching the uploaded sketch's timer API.
