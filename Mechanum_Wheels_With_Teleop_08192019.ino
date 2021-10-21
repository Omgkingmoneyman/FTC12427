/*  Program: Mechanum Wheel Demo
 *  Date: 3/2/19
 *  Author: ABL
 *  Description: This program uses the Tele-Op Module and a DC Motor Expansion Controller to control a TETRIX Robot with 4 Mechanum Wheels.
 *  Physical robot configuration is:    left front motor connected to PRIZM motor 1
 *                                      right front motor connected to PRIZM motor 2
 *                                      left back motor connected to exc motor 1
 *                                      right back motor connected to exc motor 2
 */
 
#include <PRIZM.h>      // Include the PRIZM library in sketch
#include <TELEOP.h>     // TETRIX Tele-Op Arduino Library

PRIZM prizm;            // Instantiate an object in the PRIZM class named "prizm"
EXPANSION exc;          // Instantiate an object in the EXPANSION class named "exc"
PS4 ps4;                // Instantiate an object in the PS4 class named "ps4"

float highSpeed = 1;                  // variable to set the motor percentage (100%) for turbo mode
float lowSpeed = highSpeed*.25;       // variable to set the motor percentage (25%) for crawl mode
float medSpeed = highSpeed*.40;       // variable to set the motor percentage (40%) for normal mode
float powerMultiplier = medSpeed;     // variable to set the motor power percentage (currently at medPower or 40%)
int mPowers[] = {0,0,                 // array to track motor powers for the 4 different motors (front left, front right...
                 0,0};                //                                                         back left,  back right)
                   
void setup() {
  
  prizm.PrizmBegin();                 // Initiates the PRIZM controller
  ps4.setDeadZone(LEFT,10);           // set the left joystick dead zone range to +/- 10 
  ps4.setDeadZone(RIGHT,10);          // set the right joystick dead zone range to +/- 10
  prizm.setMotorInvert(1,1);          // invert front left motor
  exc.setMotorInvert(1,1,1);          // invert back left motor
}

void loop() {
  ps4.getPS4();                                                     // get the status of the PS4 button states
  while (ps4.Button(CIRCLE)==1){                                    // while the circle button is pressed:
    ps4.getPS4();                                                     // update the status of the PS4 button states
    adjustMotorPowers();                                              // call function to adjust powerMultiplier based on what buttons are pressed
    prizm.setMotorPowers(100*powerMultiplier,-100*powerMultiplier);   // set PRIZM motor powers to turn counterclockwise
    exc.setMotorPowers(1,100*powerMultiplier,-100*powerMultiplier);   // set EXC motor powers to turn counterclockwise
  }                                                                 // end while loop for when circle button is pressed
  
  while (ps4.Button(SQUARE)==1){                                    // while the square button is pressed:
    ps4.getPS4();                                                     // update the status of the PS4 button states
    adjustMotorPowers();                                              // call function to adjust powerMultiplier based on what buttons are pressed
    prizm.setMotorPowers(-100*powerMultiplier,100*powerMultiplier);   // set PRIZM motor powers to turn clockwise
    exc.setMotorPowers(1,-100*powerMultiplier,100*powerMultiplier);   // set EXC motor powers to turn clockwise
  }                                                                 // end while loop for when square button is pressed
  
  while (ps4.Button(TRIANGLE)==1){                                  // while the triangle button is pressed:
    ps4.getPS4();                                                     // update the status of the PS4 button states
    adjustMotorPowers();                                              // call function to adjust powerMultiplier based on what buttons are pressed
    prizm.setMotorPowers(100*powerMultiplier,100*powerMultiplier);    // set PRIZM motor powers to go forward
    exc.setMotorPowers(1,100*powerMultiplier,100*powerMultiplier);    // set EXC motor powers to go forward
  }                                                                 // end while loop for when triangle button is pressed
  
  while (ps4.Button(CROSS)==1){                                     // while the cross button is pressed:
    ps4.getPS4();                                                     // update the status of the PS4 button states
    adjustMotorPowers();                                              // call function to adjust powerMultiplier based on what buttons are pressed
    prizm.setMotorPowers(-100*powerMultiplier,-100*powerMultiplier);  // set PRIZM motor powers to go backward
    exc.setMotorPowers(1,-100*powerMultiplier,-100*powerMultiplier);  // set EXC motor powers to go backward
  }                                                                 // end while loop for when cross button is pressed

  adjustMotorPowers();                                              // call function to adjust powerMultiplier based on what buttons are pressed

/* The next two lines of code set the mPowers array to the motor power values needed for the four different motors. 
 *  The array is set up in the following pattern {left front motor connected to PRIZM motor 1, right front motor connected to PRIZM motor 2,
 *                                                left back motor connected to exc motor 1, right back motor connected to exc motor 2}
 *                                                
 *  Equations are used to determine the power level of each motor based on PS4 joystick positions. Here are the equations for each motor:
 *  Front Left Motor:  (LY + LX - RX) * powerMultiplier         (Left Stick Y Position + Left Stick X Position - Right Stick X Position)
 *  Front Right Motor: (LY - LX + RX) * powerMultiplier         (Left Stick Y Position - Left Stick X Position + Right Stick X Position)
 *  Back Left Motor:   (LY - LX - RX) * powerMultiplier         (Left Stick Y Position - Left Stick X Position - Right Stick X Position)
 *  Back Right Motor:  (LY + LX + RX) * powerMultiplier         (Left Stick Y Position + Left Stick X Position + Right Stick X Position)
 *  Each motor power is then constrained between -100 and 100 so that when both sticks are used at the same time the power never exceeds 100.
 */
  
  int mPowers[] = {constrain((ps4.Motor(LY)+ps4.Motor(LX)-ps4.Motor(RX))*powerMultiplier,-100,100), constrain((ps4.Motor(LY)-ps4.Motor(LX)+ps4.Motor(RX))*powerMultiplier,-100,100),
                   constrain((ps4.Motor(LY)-ps4.Motor(LX)-ps4.Motor(RX))*powerMultiplier,-100,100), constrain((ps4.Motor(LY)+ps4.Motor(LX)+ps4.Motor(RX))*powerMultiplier,-100,100)};
  
  prizm.setMotorPowers(mPowers[0], mPowers[1]);                     // set PRIZM motor powers based on the values from the mPowers array
  exc.setMotorPowers(1,mPowers[2], mPowers[3]);                     // set EXC motor powers based on the values from the mPowers array
}

void adjustMotorPowers(){                                           // called function to change the low, med, and high speed motor powers
  if(ps4.Button(L1)==1){                                              // if the L1 button is pressed: 
    powerMultiplier = lowSpeed;                                         // set powerMultiplier to lowSpeed for crawl mode
    ps4.setRumble(SLOW);                                                // set the PS4 ruble speed to slow
    ps4.setLED(RED);}                                                   // set the game pad LED to red to indicate crawl mode
  else if(ps4.Button(R1)==1){                                         // else if the R1 button is pressed: 
    powerMultiplier = highSpeed;                                        // set the powerMultiplier to highSpeed for turbo mode
    ps4.setRumble(FAST);                                                // set the PS4 ruble speed to fast
    ps4.setLED(GREEN);}                                                 // set the game pad LED to green to indicate turbo mode
  else {                                                              // otherwise, the first two conditions are not true, so:
    powerMultiplier = medSpeed;                                         // set the powerMultiplier to medSpeed for normal mode
    ps4.setRumble(STOP);                                                // set the PS4 ruble speed to off
    ps4.setLED(YELLOW);}                                                // set the game pad LED to yellow to indicate normal mode
}                                                                   // end the called function
