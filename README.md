# Teensy-4.0-Flight-Controller
Teensy 4.0 based Flight controller using an MPU9250.
## This project was created purely for testing new ideas and software and is therefore not safe and in its current state does not work.
This project uses a modified version of Bolderflights MPU9250 library that enables rates of up to 8khz and fifo reseting for reliability. Although i have lost this modified library meaning the software will have to be slightly modified in order for it as stated above. Removing fifo resetting, limiting loop rates to 1khz and changing DLPF settings at setup should get it running. This change fisks the fifo overflowing and corrupting the data on it and all new data and also reduces performance heavily due to the reduced loop rate.

