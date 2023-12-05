/* Important Operational Notes */

1. Due to oddity in Adafruit objects (combined with my limited understanding), accelerometer & magnetometer objects are declared & intially setup in main with operations being executed outside of such. Does not seem to work, otherwise.

2. Need to find better way to calibrate A2D converter for thermocouple amp board input. Tried linear interpolation (via excel datapoint sample analysis); only accurate when away from lower end (0v) and upper end (3.3v) input values. Currently have function just taking rolling average of 10 samples.

/* /Important Operational Notes */