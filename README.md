# PHYS523
Equine RTK Project

Includes all Code going into Project for Physics 523.

-Basestation code

-RTCM Client code

Both were adapted from Sparkfun GNSS and RadioLib Library Example Codes and developed for the LoRa Things eXploraBLe Plus

Things to Test:

1. Maximum Connection distance for the current LoRa antennae - previous test ~ 0.2mi
2. Will putting the base station on the roof improve horizontal accuracy to ~ 1cm as in Sparkfun specs
3. Inductive Charging - Not Pressing
4. Usefulness of IMU - Do after GNSS System for sure works.
5. Verify RTCM data is correctly sent to GNSS system as well as which method, i2C vs UART (believed it to be but can never be too safe)
6. Adjust the code to provide data in a way that provide data that may contain actual insight into the health of the horse
