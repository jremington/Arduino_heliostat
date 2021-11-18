//sun tracker
//AZ drive, 28BYJ48 stepper plus servocity 84:16 pan platform kit
// 10698.9 steps/rev full step drive table
//ALT drive, Vexta five phase bipolar stepper
// 2000 steps/rev @ half step drive table
// S. J. Remington 7/2020

#include <SolarPosition.h>
#include <TimeLib.h>

// default globals
// FILL IN home position lat, lon in degrees
float homelat = 0.0;
float homelong = 0.0;

SolarPosition home(homelat, homelon);

// (local) time and date elements
int tH = 17; //hour local
int tM = 0;  //minute
int tS = 0;  //second
int tMo = 11; //month
int tD = 1; //day
int tYYYY = 2020; //year
int time_zone = 8; //additive correction, local to UTC

//declare variables for the motor pins
int Az_motor1 = A0;    //OUT1 Blue   - 28BYJ48 pin 1
int Az_motor2 = A1;    //OUT2 Pink   - 28BYJ48 pin 2
int Az_motor3 = A2;    //OUT3 Yellow - 28BYJ48 pin 3
int Az_motor4 = A3;    //OUT4 Orange - 28BYJ48 pin 4
int Az_step_delay = 3;  //variable to set stepper speed
// Red    - 28BYJ48 pin 5 (VCC)
// full step lookup table:
char lookup1[4] = {B01100, B00110, B00011, B01001};
// half step lookup table:
// char lookup2[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};
// lookup table indices
int Az_index = 0, Az_index_max = 4;

// Altitude motor (5-phase Vexta). See https://forum.arduino.cc/index.php?topic=708714.0
// pin assignment table: coil, {start,finish}
//                               A       B       C       D       E
uint8_t Alt_pin_table [5][2] = {{2, 3}, {4, 5}, {6, 7}, {8, 9}, {11, 12}};

// drive sequences for 5-phase motor
// half step (2000 steps/rev). Set index_max=20, skip=8
//                     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20
int coil_drive[20] = { 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};
int Alt_index_max = 20;
int skip = 8;
int permuted_indices[5]; //step table index for each coil
int Alt_index = 0; //initialize step table index to start
long Alt_step_delay = 3; //millis delay after step


int Az_steps_per_rev = 10699; //rounded up
int Alt_steps_per_rev = 2000;
float Az_degrees_per_step = 360.0 / 10698.9;
float Alt_degrees_per_step = 360.0 / 2000.0;

//assumed tracker starting position altitude, azimuth(pointing due East at horizon)
int Alt_pos = 0, Az_pos = 90.0 / Az_degrees_per_step + 0.5;

//command interpreter data
float f_arg; //command line arguments
int n_arg;

void setup() {
  //declare the motor pins as outputs
  pinMode(Az_motor1, OUTPUT);
  pinMode(Az_motor2, OUTPUT);
  pinMode(Az_motor3, OUTPUT);
  pinMode(Az_motor4, OUTPUT);
  // altitude motor
  for (int i = 0; i < 10; i++) pinMode(Alt_pin_table[i], OUTPUT);  //motor control pins

  // turn on motor coils
  set_Az_outputs();
  set_Alt_outputs();

  // set current time and date to (hours, minutes, seconds, day, month, year)
  setTime(tH, tM, tS, tD, tMo, tYYYY);

  Serial.begin(9600);
  Serial.println(F("pan/tilt platform initializing"));
  // remove initial backlash in azimuth drive
  for (int i = 0; i < 100; i++) step_Az_minus();
  for (int i = 0; i < 100; i++) step_Az_plus();
  Serial.print(F("Default alt/az: ")); Serial.print(Alt_pos * Alt_degrees_per_step);
  Serial.print(F(" at step ")); Serial.print(Alt_pos);
  Serial.print(F(" / ")); Serial.print(Az_pos * Az_degrees_per_step);
  Serial.print(F(" at step ")); Serial.println(Az_pos);
  printTime();
  Serial.println(F(">"));
}

//
// loop() has two modes
//  (1) get and execute position commands from serial monitor
//  (2) track sun
//
// state machine defs
#define CMD_INPUT 0
#define TRACKING 1

void loop() {

  char c, OK;
  static char cmdBuffer[20];  //serial command input buffer
  static byte cmdIndex = 0;
  static unsigned long start_time_ms = 0;
  static unsigned int state = CMD_INPUT;
  static float salt, saz, nalt, naz, talt, taz;

  float Sxyz[3], Nxyz[3], Txyz[3]; //mirror tracking vectors, sun, mirror normal, target

  //
  // state == TRACKING.
  // Update mirron normal (alt, az) every minute, check for serial input to break
  // T, target vector, calculated previously (normalized)
  // S, sun vector, calculated on the fly
  //
  if (state == TRACKING) {
    if (millis() - start_time_ms > 60UL * 1000UL) {  //every minute do something
      start_time_ms = millis();
      printTime(); //print the current local time
      time_t t_now = now() + time_zone * 3600UL; //correct to UTC

      // sun tracking
      //      move_to_altitude(home.getSolarElevation(t_now));
      //      move_to_azimuth(home.getSolarAzimuth(t_now));

      // mirror pointing
      Serial.print(F("Sun alt/az "));
      Serial.print(salt = home.getSolarElevation(t_now));
      Serial.print(F(" "));
      Serial.println(saz = home.getSolarAzimuth(t_now));

      // Sun too low?
      if (salt < 3.0) {
        Serial.println(F("Sun too low, aborted"));
        Serial.println(F(">"));
        state = CMD_INPUT;
      }

      // get sun vector
      altAz_to_xyz(salt, saz, Sxyz);
      //      Serial.print("S: "); Serial.print(Sxyz[0], 3);
      //      Serial.print(" "); Serial.print(Sxyz[1], 3);
      //      Serial.print(" "); Serial.println(Sxyz[2], 3);

      // mirror alt,az = vector sum Sun + Target
      Nxyz[0] = Sxyz[0] + Txyz[0];
      Nxyz[1] = Sxyz[1] + Txyz[1];
      Nxyz[2] = Sxyz[2] + Txyz[2];
      //      Serial.print("N: "); Serial.print(Nxyz[0], 3);
      //      Serial.print(" "); Serial.print(Nxyz[1], 3);
      //      Serial.print(" "); Serial.println(Nxyz[2], 3);
      xyz_to_altaz(&nalt, &naz, Nxyz);
      Serial.print(F("Mirror normal alt, az ")); Serial.print(nalt);
      Serial.print(F(" ")); Serial.println(naz);
      move_to_altitude(nalt);
      move_to_azimuth(naz);
    }
    if (Serial.available()) { //check for input
      state = CMD_INPUT;
      Serial.println(F("Command Input\r>"));
      cmdIndex = 0;
      cmdBuffer[0] = 0; //clear input buffer
      OK = 1;
    }
  }

  //
  // state == CMD_INPUT
  // check for command input of form xy nnnn  (nnnn float or integer, blank optional)
  //
  if (state == CMD_INPUT && Serial.available())  {
    c = Serial.read();
    if (cmdIndex < sizeof(cmdBuffer) - 1) cmdBuffer[cmdIndex++] = c; //avoid overflow

    //command terminated?
    if (c == '\n' || c == '\r') {
      cmdBuffer[cmdIndex - 1] = 0; //terminate string
      f_arg = atof(&cmdBuffer[2]); //ignore two leading alpha characters, get first argument
      n_arg = f_arg;
      OK = 0; //reset command OK flag

      // COMMAND DECODER
      
      //
      // t hh:mm  input local time, convert to UTC and print updated local time
      // t (no argument) print current local time
      //
      
      if (cmdBuffer[0] == 't') {
        char * pch = strchr(cmdBuffer, ':');
        if (pch != NULL) { //got an expected delimiter
          tH = atoi(&cmdBuffer[1]); //get the hour
          tM = atoi(pch + 1);
          // set current time and date to (hours, minutes, seconds, day, month, year)
          setTime(tH, tM, tS, tD, tMo, tYYYY);
          Serial.print(F("New local time "));
        }
        printTime(); //if no delimiter, just print the current local time
        time_t t_now = now() + time_zone * 3600UL; //correct to UTC
        Serial.print(F("Sun alt/az "));
        Serial.print(home.getSolarElevation(t_now));
        Serial.print(F(" "));
        Serial.println(home.getSolarAzimuth(t_now));
        OK = 1;  //interpretable command
      }  //end t hh:mm

      //
      // d mm/dd  input UTC date (slash required)
      //
      if (cmdBuffer[0] == 'd') {
        char * pch = strchr(cmdBuffer, '/');
        if (pch != NULL) { //expected delimiter found
          OK = 1;
          tMo = atoi(&cmdBuffer[1]); //get month
          tD = atoi(pch + 1); //set day
          // set current local time and date to (hours, minutes, seconds, day, month, year)
          setTime(tH, tM, tS, tD, tMo, tYYYY);
          Serial.print(F("New local date ")); printTime();
        }
      } //end d dd/mm

      //
      // ja  NNNN
      // jz  NNNN -> jog alt or az by (+/-)NNNN steps, for fine positioning
      //
      if (cmdBuffer[0] == 'j') {

        if (cmdBuffer[1] == 'a') { //ja
          if (n_arg < 0)
            for (int i = 0; i < abs(n_arg); i++) step_Alt_minus();
          else
            for (int i = 0; i < n_arg; i++) step_Alt_plus();
          Serial.print(F("Jog alt to "));
          Serial.print(Alt_pos * Alt_degrees_per_step);
          Serial.print(F(", step pos ")); Serial.println(Alt_pos);
          OK = 1;
        }
        if (cmdBuffer[1] == 'z') { //jz
          if (n_arg < 0) {
            n_arg = abs(n_arg);
            for (int i = 0; i < n_arg + 100; i++) step_Az_minus(); //anti backlash move
            for (int i = 0; i < 100; i++) step_Az_plus();
            OK = 1;
          }
          else {
            for (int i = 0; i < n_arg; i++) step_Az_plus();
          }
          Serial.print(F("Jog az to "));
          Serial.print(Az_pos * Az_degrees_per_step);
          Serial.print(F(", step pos ")); Serial.println(Az_pos);
          OK = 1;
        }
      } //end jog a/z

      //
      // sa, sz  -> Set current position to altitude/azimuth. Resets stepper origin
      //
      if (cmdBuffer[0] == 's') {

        if (cmdBuffer[1] == 'a') { //sa
          Alt_pos = (f_arg / Alt_degrees_per_step) + 0.5; //round to nearest positive value
          Serial.print(F("Reset alt to "));
          Serial.print(Alt_pos * Alt_degrees_per_step);
          Serial.print(F(", current step reset to ")); Serial.println(Alt_pos);
          OK = 1;
        }

        if (cmdBuffer[1] == 'z') { //sz
          Az_pos = (f_arg / Az_degrees_per_step) + 0.5; //round to nearest positive value
          Serial.print(F("Reset az to "));
          Serial.print(Az_pos * Az_degrees_per_step);
          Serial.print(F(", current step reset to ")); Serial.println(Az_pos);
          OK = 1;
        }
      } //end set a/z command

      //
      // ma NN.NN, mz NN.NN move to position alt, az in degrees (floating point)
      // checks for disallowed angles, installation specific
      //
      if (cmdBuffer[0] == 'm') {

        if (cmdBuffer[1] == 'a') {
          if (f_arg < 135.0 && f_arg > -135.0) move_to_altitude(f_arg); //avoid hitting support
          OK = 1;
        }

        if (cmdBuffer[1] == 'z') {
          if (f_arg < 0.0) f_arg += 360.0; //compass circle
          move_to_azimuth(f_arg);
          OK = 1;
        }
      } //end m a/z command

      //
      // 'a' Track sun from now
      // update alt,az every minute, set state to TRACKING, to execute platform motions
      // any character on serial input terminates tracking
      //

      if (cmdBuffer[0] == 'a') {

        Serial.println(F("\nSet up for tracking..."));
        nalt = Alt_pos * Alt_degrees_per_step;
        naz = Az_pos * Az_degrees_per_step;
        Serial.print(F("Current alt, az ")); Serial.print(nalt);
        Serial.print(F(" ")); Serial.println(naz);

        // get mirror normal unit vector from current alt,az
        altAz_to_xyz(nalt, naz, Nxyz);
        Serial.print(F("N: ")); Serial.print(Nxyz[0], 3);
        Serial.print(F(" ")); Serial.print(Nxyz[1], 3);
        Serial.print(F(" ")); Serial.println(Nxyz[2], 3);

        time_t t_now = now() + time_zone * 3600UL; //correct to UTC
        Serial.print(F("Sun alt, az "));
        Serial.print(salt = home.getSolarElevation(t_now));
        Serial.print(F(" "));
        Serial.println(saz = home.getSolarAzimuth(t_now));

        // get sun unit vector
        altAz_to_xyz(salt, saz, Sxyz);
        Serial.print("S: "); Serial.print(Sxyz[0], 3);
        Serial.print(" "); Serial.print(Sxyz[1], 3);
        Serial.print(" "); Serial.println(Sxyz[2], 3);

        // magnitude of N (mirror normal) = 2 (S dot N)/Mag(S)/Mag(N)
        // here both vectors are already normalized, so mag(x) = 1.0
        float m = 2.0 * (Sxyz[0] * Nxyz[0] + Sxyz[1] * Nxyz[1] + Sxyz[2] * Nxyz[2]);
        Serial.print("mag N "); Serial.println(m);

        //apply magnitude to mirror normal vector
        for (int i = 0; i < 3; i++) Nxyz[i] *= m;

        Serial.print(F("N: ")); Serial.print(Nxyz[0], 3);
        Serial.print(F(" ")); Serial.print(Nxyz[1], 3);
        Serial.print(F(" ")); Serial.println(Nxyz[2], 3);
        xyz_to_altaz(&nalt, &naz, Nxyz);
        Serial.print(F("Mirror normal alt, az ")); Serial.print(nalt);
        Serial.print(F(" ")); Serial.println(naz);

        //get target vector
        Txyz[0] = Nxyz[0] - Sxyz[0];
        Txyz[1] = Nxyz[1] - Sxyz[1];
        Txyz[2] = Nxyz[2] - Sxyz[2];
        Serial.print(F("T: ")); Serial.print(Txyz[0], 3);
        Serial.print(F(" ")); Serial.print(Txyz[1], 3);
        Serial.print(F(" ")); Serial.println(Txyz[2], 3);
        xyz_to_altaz(&talt, &taz, Txyz);
        Serial.print(F("Target alt, az: ")); Serial.print(talt);
        Serial.print(F(" ")); Serial.println(taz);

        Serial.println(F("\nTracking..."));
        state = TRACKING;
        
        // flush serial input buffer
        while (Serial.available() > 0) Serial.read();
        start_time_ms = millis();
        OK = 1;  //no input error
      }
      
     // Command Error
      if (!OK) { //command not understood
        Serial.print("?> ");
        Serial.println(cmdBuffer);
        Serial.print(F(">")); //ready for next command
      }
      cmdIndex = 0; //clear input buffer for next command
    } //end if (cr or lf)

  }  //end if serial.available()

} //end loop()


// USEFUL FUNCTIONS

//
// form unit vector from v[]
//
void normalize(float v[3]) {
  float m = 0;
  for (int i = 0; i < 3; i++) m += v[i] * v[i];
  m = sqrt(m);
  for (int i = 0; i < 3; i++) v[i] /= m;
}
//
// (alt,az) in degrees to unit vector xyz (y North, x East, Z up)
//
void altAz_to_xyz(float alt, float az, float xyz[3]) {
  float dtr = PI / 180.0;
  *xyz++ = cos(alt * dtr) * sin(az * dtr);
  *xyz++ = cos(alt * dtr) * cos(az * dtr);
  *xyz = sin(alt * dtr);
}
//
// vector xyz (y North, x East, Z up) to (alt, az) in degrees
//
void xyz_to_altaz(float * alt, float * az, float xyz[3]) {
  float rtd = 180.0 / PI;
  float v[3], m = 0;
  // normalize input vector
  for (byte i = 0; i < 3; i++) m += xyz[i] * xyz[i];
  m = sqrt(m);
  for (byte i = 0; i < 3; i++) v[i] = xyz[i] / m;
  float t = rtd * atan2(v[0], v[1]);
  if (t < 0.0) t += 360.0; //compass wrap
  *az = t;
  *alt = rtd * asin(v[2]);
}

void printTime(void) // time_t t)
{
  char buf[22];
  sprintf(buf, "%02d/%02d/%4d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
  Serial.println(buf);
}
// move to ALT

void move_to_altitude (float alt) {
  Serial.print(F("Move alt to "));  //degrees
  Serial.print(alt);
  int n = (alt / Alt_degrees_per_step) + 0.5; //round to nearest step
  int steps = n - Alt_pos;
  Serial.print(F(", steps to go ")); Serial.print(abs(steps));
  if (steps < 0)
    for (int i = 0; i < abs(steps); i++) step_Alt_minus();
  else
    for (int i = 0; i < steps; i++) step_Alt_plus();
  Serial.print(F(". Now at step ")); Serial.println(Alt_pos);
}

// move to AZ

void move_to_azimuth(float azimuth) {
  Serial.print(F("Move az to "));
  Serial.print(azimuth);
  int n = (azimuth / Az_degrees_per_step) + 0.5; //round to nearest step
  int steps = n - Az_pos;
  Serial.print(F(", steps to go ")); Serial.print(abs(steps));
  if (steps < 0) {
    for (int i = 0; i < abs(steps) + 100; i++) step_Az_minus(); //anti backlash move
    for (int i = 0; i < 100; i++) step_Az_plus();
  }
  else for (int i = 0; i < steps; i++) step_Az_plus();
  Serial.print(F(". Now at step ")); Serial.println(Az_pos);
}

//
// azimuth step +/-
//
void step_Az_plus()
{
  Az_index++;
  if (Az_index >= Az_index_max) Az_index = 0;
  set_Az_outputs();
  delay(Az_step_delay);
  Az_pos++;
  if (Az_pos >= Az_steps_per_rev) Az_pos -= Az_steps_per_rev; //limit to full circle
}
void step_Az_minus()
{
  Az_index--;
  if (Az_index < 0) Az_index += Az_index_max;  //cycle round
  set_Az_outputs();
  delay(Az_step_delay);
  Az_pos--;
  if (Az_pos < 0) Az_pos += Az_steps_per_rev; //positive full circle
}

// apply power to windings for current step index
// use full step table for azimuth
void set_Az_outputs(void)
{
  digitalWrite(Az_motor1, bitRead(lookup1[Az_index], 0));
  digitalWrite(Az_motor2, bitRead(lookup1[Az_index], 1));
  digitalWrite(Az_motor3, bitRead(lookup1[Az_index], 2));
  digitalWrite(Az_motor4, bitRead(lookup1[Az_index], 3));
}

// apply power to windings for current step index
void set_Alt_outputs(void) {
  int k;
  if (Alt_index >= Alt_index_max) Alt_index = 0;
  if (Alt_index < 0 )Alt_index += Alt_index_max;
  permuted_indices[0] = Alt_index;
  permuted_indices[1] = (permuted_indices[0] + skip) % Alt_index_max;
  permuted_indices[2] = (permuted_indices[1] + skip) % Alt_index_max;
  permuted_indices[3] = (permuted_indices[2] + skip) % Alt_index_max;
  permuted_indices[4] = (permuted_indices[3] + skip) % Alt_index_max;
  for (int j = 0; j < 5; j++) { //loop over coils
    k = permuted_indices[j];
    if (coil_drive[k] > 0) {
      digitalWrite(Alt_pin_table[j][0], 1); // plus
      digitalWrite(Alt_pin_table[j][1], 0);
    }
    if (coil_drive[k] == 0) {
      digitalWrite(Alt_pin_table[j][0], 0); // off
      digitalWrite(Alt_pin_table[j][1], 0);
    }
    if (coil_drive[k] < 0) {
      digitalWrite(Alt_pin_table[j][0], 0); // minus
      digitalWrite(Alt_pin_table[j][1], 1);
    }
  } // end for j
}

// ALT motor step +/-
void step_Alt_plus(void) {  //CW
  Alt_index++; //next call corrects index, if required
  set_Alt_outputs();
  delay(Alt_step_delay);
  Alt_pos++;
  if (Alt_pos > Alt_steps_per_rev) Alt_pos -= Alt_steps_per_rev; //limit to full circle
}
void step_Alt_minus(void) {  //CCW
  Alt_index--; //next call corrects index, if required
  set_Alt_outputs();
  delay(Alt_step_delay);
  Alt_pos--;
  if (Alt_pos < 0) Alt_pos += Alt_steps_per_rev; //limit to full circle
}
