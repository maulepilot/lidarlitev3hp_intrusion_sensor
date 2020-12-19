# lidarlitev3hp_intrusion_sensor
Garmin LIDAR-Lite v3HP sensor used with Arduino MKR WiFi 1010 as intrusion sensor
Uses a modified version of the LIDARLite_Arduino_Library-master library with #define LEGACY_I2C in LIDARLite_v3HP.cpp and LIDARLite_v4LED.cpp. The sketch will not compile on the MKR WiFi 1010 otherwise.
Initial provision is made in the sketch for pan and tilt servos, but the lidar measurements are not stable enough between pulses to prevent false alarms, so servo operation is still under experimentation.
