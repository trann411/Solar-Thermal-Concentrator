/* Intial Pin Mapping */
int GPIO0 = 5;
int GPIO1 = 6;
int GPIO2 = 7;
int GPIO3 = 8;
int GPIO4 = 9;
int GPIO5 = 10;
int GPIO6 = 11;
int GPIO7 = 12;
int GPIO8 = 13;
int GPIO9 = 14;
int GPIO10 = 15;
int GPIO11 = 16;
int GPIO12 = 17;
int GPIO13 = 18;
int GPIO14 = 19;
int XTAL_32K_P = 21;
int XTAL_32K_N = 22;
int GPIO17 = 23;
int GPIO18 = 24;
int GPIO19 = 25;
int GPIO20 = 26;
int GPIO21 = 27;
int SPICS1 = 28;
int SPIHD = 30;
int SPIWP = 31;
int SPICS0 = 32;
int SPICLK = 33;
int SPIQ = 34;
int SPID = 35;
int SPICLK_N = 36;
int SPICLK_P = 37;
int GPIO33 = 38;
int GPIO34 = 39;
int GPIO35 = 40;
int GPIO36 = 41;
int GPIO37 = 42;
int GPIO38 = 43;
int MTCK = 44;
int MTDO = 45;
int MTDI = 47;
int MTMS = 48;
int U0TXD = 49;
int U0RXD = 50;
int GPIO45 = 51;
int GPIO46 = 52;
int XTAL_N = 53;
int XTAL_P = 54;
/* /Intial Pin Mapping */

/* Azimuth Rotary Encoder (ARE) Pin Assignment & State Variable */
int ARE_APulse = GPIO20;
int ARE_ANotPulse = GPIO21;
int ARE_BPulse = GPIO33;
int ARE_BNotPulse = GPIO34;
//---
int ARE_APulse_State = 0;
int ARE_ANotPulse_State = 0;
int ARE_BPulse_State = 0;
int ARE_BNotPulse_State = 0;
/* /Azimuth Rotary Encoder (ARE) Pin Assignment & State Variable */

/* Dual Tilt/Azimuth Motor Controller (DTAC) Pin Assignment */
int DTAC_PWM1 = GPIO6;
int DTAC_PWM2 = GPIO7;
int DTAC_IN1 = GPIO8;
int DTAC_IN2 = GPIO9;
int DTAC_IN3 = GPIO10;
int DTAC_IN4 = GPIO11;
/* /Dual Tilt/Azimuth Motor Controller (DTAC) Pin Assignment */

/* Pyranometer (PYRO) Pin Assignment & Value Variable */
int PYRO_AnalogIn = GPIO12;
//---
double PYRO_AnalogIn_Value = 0;
/* /Pyranometer (PYRO) Pin Assignment & Value Variable */

/* Shutter Actuator Pin (SAC) Pin Assignment */
int SAC_PWM1 = GPIO0;
int SAC_PWM2 = GPIO1;
int SAC_IN1 = GPIO2;
int SAC_IN2 = GPIO3;
int SAC_IN3 = GPIO4;
int SAC_IN4 = GPIO5;
/* /Shutter Actuator Pin (SAC) Pin Assignment */

/* Shutter A Rotary Encoder (SARE) Pin Assignment & State Variable*/
int SARE_APulse = GPIO14;
int SARE_BPulse = GPIO17;
//---
int SARE_APulse_State = 0;
int SARE_BPulse_State = 0;
/* /Shutter A Rotary Encoder (SARE) Pin Assignment */

/* Shutter B Rotary Encoder (SBRE) Pin Assignment */
int SBRE_APulse = GPIO18;
int SBRE_BPulse = GPIO19;
//---
int SBRE_APulse_State = 0;
int SBRE_BPulse_State = 0;
/* /Shutter B Rotary Encoder (SBRE) Pin Assignment */

/* Thermocouple (THERMO) Pin Assignment */
int THERMO_AnalogIn = GPIO13;
//---
double THERMO_AnalogIn_Value = 0;
/* /Thermocouple (THERMO) Pin Assignment */

/* Miscellaneous Variables */
bool writeMovement = false;
bool readSensors = false;
/* /Miscellaneous Variables */


void setup() {
  // put your setup code here, to run once:

  /* Set Mode for ARE Pins*/
  pinMode(ARE_APulse, INPUT);
  pinMode(ARE_ANotPulse, INPUT);
  pinMode(ARE_BPulse, INPUT);
  pinMode(ARE_BNotPulse, INPUT);
  /* /Set Mode for ARE Pins */

  /* Set Mode for DTAC Pins */
  pinMode(DTAC_PWM1, OUTPUT);
  pinMode(DTAC_PWM2, OUTPUT);
  pinMode(DTAC_IN1, OUTPUT);
  pinMode(DTAC_IN2, OUTPUT);
  pinMode(DTAC_IN3, OUTPUT);
  pinMode(DTAC_IN4, OUTPUT);
  /* /Set Mode for DTAC Pins */

  /* Set Mode for PYRO Pins */
  pinMode(PYRO_AnalogIn, INPUT);
  /* /Set Mode for PYRO Pins */

  /* Set Mode For SAC Pins */
  pinMode(SAC_PWM1, OUTPUT);
  pinMode(SAC_PWM2, OUTPUT);
  pinMode(SAC_IN1, OUTPUT);
  pinMode(SAC_IN2, OUTPUT);
  pinMode(SAC_IN3, OUTPUT);
  pinMode(SAC_IN4, OUTPUT);
  /* /Set Mode For SAC Pins* */

  /* Set Mode for SARE Pins */
  pinMode(SARE_APulse, INPUT);
  pinMode(SARE_BPulse, INPUT);
  /* /Set Mode for SARE Pins */

  /* Set Mode for SBRE Pins */
  pinMode(SBRE_APulse, INPUT);
  pinMode(SBRE_BPulse, INPUT);
  /* /Set Mode for SBRE Pins */

  /* Set Mode for THERMO Pins */
  pinMode(THERMO_AnalogIn, INPUT);
  /* /Set Mode for THERMO Pins */
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.begin(9600); //Debugging Interface

  if (writeMovement == true) {
    // *Execute desired movement*
    delay (3000); // Wait for rig movement to settle
  }

  if (readSensors == true) {
    /* Query ARE */
    ARE_APulse_State = digitalRead(ARE_APulse);
    ARE_ANotPulse_State = digitalRead(ARE_ANotPulse);
    ARE_BPulse_State = digitalRead(ARE_BPulse);
    ARE_BNotPulse_State = digitalRead(ARE_BNotPulse);
    /* /Query ARE */

    /* Query PYRO */
    PYRO_AnalogIn_Value = analogRead(PYRO_AnalogIn);
    /* /Query PYRO */

    /* Query SARE */
    SARE_APulse_State = digitalRead(SARE_APulse);
    SARE_BPulse_State = digitalRead(SARE_BPulse);
    /* /Query SARE */

    /* Query SBRE */
    SBRE_APulse_State = digitalRead(SBRE_APulse);
    SBRE_BPulse_State = digitalRead(SBRE_BPulse);
    /* /Query SBRE */

    /* Query THERMO */
    THERMO_AnalogIn_Value = analogRead(THERMO_AnalogIn);
    /* /Query THERMO */

    /* Post-Process AnalogIn Values */
    // Insert PYRO equation
    THERMO_AnalogIn_Value = (THERMO_AnalogIn_Value - 1.25) / 0.005;
    /* /Post-Process AnalogIn Values */
  }
  
  // Push Results to MATLAB
  // Rinse & Repeat
}
