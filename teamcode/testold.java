/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="testold", group="moveside")

public class testold extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareFile robot           = new HardwareFile();    // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
    double stopVar = 0.00;
    float flMovement;
    float blMovement;
    float frMovement;
    float brMovement;
    @Override
    public void runOpMode() {
        //variables are here
        int funcStop;
        //left and rightblock  for side to side of arm movement variable
        float blockLeft_Right;
        float blockUp_Down;
        float blockspeed;
        double stosMovement;
        float turn;
        float driveMovement;
        float strafe;
        double servoPosition;
        double servoPositionLeft = 1;
        double servoPositionRight = 0;
        double deadZone = .2;
        float updownSpeed = 0.50f;
        float sidetosideSpeed = 0.05f;
        double correctionValue = 0.6;
        // gives the motors that need it more power
        float powerCorrection = 3.0f;
        //float variables must be a have a f at end or else they are considered double
        float slowFactor = 1.00f;
        double stallPower = 0.05;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.backLeft_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //slowing down for concise controll
           //    stopFunc();

            //slowing it down or speeding it up code
            slowFactor= 1.00f;
          if(gamepad1.left_bumper){
                slowFactor=1.00f;
            } else if (gamepad1.right_bumper) {
           slowFactor=0.50f;
          } else  {
              slowFactor= 0.30f;
              //add 2nd slow factor here
          }
          //speed block mover will be set too and if statement in future
            if (gamepad2.left_bumper){
                sidetosideSpeed = 0.5f;
                updownSpeed = 0.3f;
            } else {
                 sidetosideSpeed = 1.0f;
                 updownSpeed = 0.6f;
            }




            //starting code for grabbing values to send to bot for movement
            //foward and backward variable, remember it is backwards so negated
            //getting values
            //gamesticks are backwards up is x and side to side is Y
            driveMovement = gamepad1.left_stick_x * slowFactor;
            turn = -gamepad1.left_stick_y * slowFactor;
            strafe = -gamepad1.right_stick_x * slowFactor;
            blockLeft_Right = gamepad2.left_stick_y * sidetosideSpeed;
            blockUp_Down= -gamepad2.right_stick_y * updownSpeed;

            //using the math
            //stop low values code
            //driveMovement = Range.clip(driveMovement, "-.8",".8");

            frMovement = driveMovement - strafe - turn;
            flMovement = driveMovement - strafe + turn;
            blMovement = driveMovement + strafe + turn;
            brMovement = driveMovement + strafe - turn;
            //movement grabber
           float blockLeft_Right_Value = blockLeft_Right;
           float upDownSpeed = updownSpeed;
            //func keeps strafe values good

           //rangeControl();
            //noticed that when vars plugged into motor no stafre but when plugged directly into motor it worked.
                //putting in a statement for slowdown

           /* if((((gamepad1.left_bumper = false) && (flMovement > deadZone) || (frMovement > deadZone)||(brMovement > deadZone)||(blMovement > deadZone)||
                    (flMovement < -deadZone) || (frMovement < -deadZone)||(brMovement < -deadZone)||(blMovement < -deadZone))))
            {

                robot.frontLeft_drive.setPower(flMovement);
                robot.frontRight_Drive.setPower(frMovement);
                robot.backRight_Drive.setPower(brMovement);
                robot.backLeft_Drive.setPower(blMovement);
            }
                elseif (gamepad1.left_bumper = true){
                robot.frontLeft_drive.setPower(flMovement);
                robot.frontRight_Drive.setPower(frMovement);
                robot.backRight_Drive.setPower(brMovement);
                robot.backLeft_Drive.setPower(blMovement);
            };

            */
            robot.frontLeft_drive.setPower(flMovement);
            robot.frontRight_Drive.setPower(frMovement);
            robot.backRight_Drive.setPower(brMovement);
            robot.backLeft_Drive.setPower(blMovement);
            //statements for moving arm might add differnt function in future
            //movement statements of block and grab servos

            //platform servo
            if (gamepad2.y) {
                robot.platformServo.setPosition(servoPositionLeft);
            }else if (gamepad2.a){
                robot.platformServo.setPosition(servoPositionRight);
            }





            //setting power to motors
            //robot.frontLeft_drive.setPower(driveMovement - strafe + turn);
            //robot.frontRight_Drive.setPower(driveMovement - strafe - turn);
            //robot.backLeft_Drive.setPower(driveMovement + strafe + turn);
            //robot.backRight_Drive.setPower(driveMovement + strafe - turn);


             /*
             robot.frontLeft_drive.setPower(flMovement);
            robot.frontRight_Drive.setPower(frMovement);
            robot.backLeft_Drive.setPower(blMovement);
            robot.backRight_Drive.setPower(brMovement);
            */


            double sidepower = blockUp_Down;
            // added data values for view on phone
            telemetry.addData("driveMovment","%.2f",driveMovement);
            telemetry.addData("strafe","%.2f",strafe);
            telemetry.addData("turn","%.2f",turn);

            telemetry.addData("fr","%.2f",frMovement);
            telemetry.addData("fl","%.2f",flMovement);
            telemetry.addData("br","%.2f",brMovement);
            telemetry.addData("bl","%.2f",blMovement);
            telemetry.addData("Updown","&.2f",blockUp_Down);
            telemetry.addData("sidetoside","&.2f",sidepower);
            telemetry.update();
        }

    }
    private void stopFunc() {

        robot.frontLeft_drive.setPower(stopVar);
        robot.frontRight_Drive.setPower(stopVar);
        robot.backRight_Drive.setPower(stopVar);
        robot.backLeft_Drive.setPower(stopVar);
    }
    public void rangeControl(){

        Range.clip(frMovement,-1,1);
        Range.clip(brMovement,-1,1);
        Range.clip(blMovement,-1,1);
        Range.clip(flMovement,-1,1);
    }
};
