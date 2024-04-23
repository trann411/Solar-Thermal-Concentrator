/* Preface */

I - Jacob Royal - want to clarify that I am (first and foremost) an Electrical Engineering student, and that I am not truthfully very 'savvy' with software development and coding. I have created (to the extent of my limited ability) the best possible product I can, but there very likely are numerous optimizations/better coding practices that could be made to my scripts.

/* /Preface */

/* Important Operational Notes */

1. Due to oddity in Adafruit objects (combined with my limited understanding), accelerometer & magnetometer objects are declared & intially setup in main with operations being executed outside of such. Does not seem to work, otherwise.

2. Need to find better way to calibrate A2D converter for thermocouple amp board input. Tried linear interpolation (via excel datapoint sample analysis); only accurate when away from lower end (0v) and upper end (3.3v) input values. Currently have function just taking rolling average of 10 samples.

3. There are an oddly high number of instances where code will compile with no errors, but the ESP32 will just reject it upon a reset and will lock itself into CircuitPython mode. 
	* The ESP32 just does not like it when you try to put any modifiers in conditional statements for any kind 	of loop or if block. Cannot use &&, ||, or any other function (i.e. abs(), etc.) within such statements.
	* Incorretly using Serial.print(), Serial.println() statements will cause this lock.
	* Neglecting to put a return statement in a function that returns a variable does not cause the Arduino 	IDE to flag an error and will instead just cause the ESP32 to become stuck in that function.

/* /Important Operational Notes */