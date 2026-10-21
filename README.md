# ESP32_CW_decoder
A CW decoder running on ESP32 display board and a microphone. 

This project is based on simplest hardware setup to demonstrate CW decoder, which works on completely different principal.
Several solutions from others use Goertzel or FFT method to detect dids and dats from Morse tone.
I am using here the SSTV library prepared by Jon Dawson (101 Things). Library originally  used on RPi Pico for SSTV decoder.
I wanted to try out this idea, because you may get precise information of tone frequency and its amplitude faster. Ploting this two information,
during decoding user can tune the radio to exact wanted tone pitch. In my case this is hard coded to 800Hz, but could be changed by adding a setup page before decoding start.
I explained more about using SSTV library in my recent video of which I will put the link below.
Hardware needed: A display board and analog microphone with integrated amplifier as on picture.

<img width="336" height="188" alt="hardware1" src="https://github.com/user-attachments/assets/b6add7e4-fe5b-4730-8875-98c740e53386" />

1x CYD ESP32-1732S019 + 1x MAX4466 Electret Microphone Amplifier with Adjustable Gain

Project will compile in Arduino with ESP32 core 2.0.17.
How to use: Just place it near your radio speaker, adjust the volume and tune pitch to align two markers.

<img width="346" height="224" alt="markers" src="https://github.com/user-attachments/assets/c79daf94-95c0-407c-9444-5840868c336d" />

YouTube video link:
My thanks here to Jonathan P Dawson and his SSTV library, which made this project possible.

Licence: Apache License 2.0
You can buy me a coffee: https://ko-fi.com/stojanm


