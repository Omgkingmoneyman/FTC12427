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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
//edited for ultimate challenge

public class HardwareFile
{
    /* Public OpMode members. */
    //new motors installed this year, changed name for back and front motor drive
    public DcMotor frontLeft_drive = null;
    public DcMotor frontRight_Drive = null;
    public DcMotor backLeft_Drive = null;
    public DcMotor backRight_Drive = null;
    public DcMotor ringMotor1 = null;
    public DcMotor ringMotor2  = null;
    // public static DistanceSensor sensorRange;
    Servo wobbleServo;
   // Servo blockServo;
   // Servo blockServo2;
    Servo platformServo;
    //servo removed here
    BNO055IMU imu;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwareFile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft_drive = hwMap.get(DcMotor.class, "Fleft.Drive");
        frontRight_Drive = hwMap.get(DcMotor.class, "Fright.Drive");
        backLeft_Drive  = hwMap.get(DcMotor.class,"Bleft.Drive");
        backRight_Drive = hwMap.get(DcMotor.class,"Bright.Drive");
        ringMotor1 = hwMap.get(DcMotor.class, "ringMotor1");
        ringMotor2 = hwMap.get(DcMotor.class,"ringMotor2");
     //removed block down drive 11-3-20
        //sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
       wobbleServo = hwMap.get(Servo.class, "wobble.Servo");
       // blockServo = hwMap.get(Servo.class, "block.Servo");
       // blockServo2 = hwMap.get(Servo.class,"grab2.Servo");
        platformServo = hwMap.get(Servo.class, "platformServo");
        imu = hwMap.get(BNO055IMU.class,"imu");
        /*
        commented out not sure what directions motors are need for what
        frontLeft_drive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRight_Drive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        */
        backRight_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       //removed block down 11/3
        // Set all motors and servos to zero power/postion
        frontLeft_drive.setPower(0);
        frontRight_Drive.setPower(0);
        backLeft_Drive.setPower(0);
        backRight_Drive.setPower(0);
        ringMotor1.setPower(0);

        ringMotor2.setPower(0);
        wobbleServo.setPosition(0);
        //blockServo.setPosition(0);
       // blockServo2.setPosition(1);
        platformServo.setPosition(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ringMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ringMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //blockDownMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
       // leftClaw  = hwMap.get(Servo.class, "left_hand");
      //  rightClaw = hwMap.get(Servo.class, "right_hand");
      // leftClaw.setPosition(MID_SERVO);
      //  rightClaw.setPosition(MID_SERVO);
    }
 }