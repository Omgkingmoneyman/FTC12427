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
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import java.util.Locale;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//@Disabled
public class AutonomousFunctions   {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    Orientation lastAngles = new Orientation();
    Orientation angles;
    Acceleration gravity;
    // State used for updating telemetry

    double globalAngle,  correction;


    LinearOpMode LO = new LinearOpMode() {
        @Override

        public void runOpMode() throws InterruptedException {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            BNO055IMU imu;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; ;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            robot.imu.initialize(parameters);
            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

        }
    };


    public ElapsedTime totalRuntime = new ElapsedTime();
    public ElapsedTime stepTime = new ElapsedTime();




    /* Constructor */
     AutonomousFunctions () {
         double botSpeed = 0.4;
         // ElapsedTime totalRuntime = new ElapsedTime();
         // ElapsedTime stepTime = new ElapsedTime();

    }




        //variables for the Af fucntions
    double botSpeed = 0.4;
    double correctionFactor = 1.3;
    double leftStrafe = -1;
    double rightStrafe = 1;
    double Forward = -1;
    double backward = 1;
    double circumfernce = 9.42;
    static final double    COUNTS_PER_MOTOR_REV   = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV ) / (WHEEL_DIAMETER_INCHES * 3.1415);



    //all functions  are down here
    public void  moveforwardbackwards(int prewaitTime, double botSpeed, double botDirection, double Distance) {
        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double flPower, frPower, blPower, brPower, correction;



        stepTime.reset();
        //finding the amount of ticks needed to move based off what is inputed
        double DistanceINCH = (COUNTS_PER_INCH) * (Distance);
        int motorticks = (int)Math.round(DistanceINCH);

        LO.sleep(prewaitTime);
        //inputing amount of ticks needed to move then setting power to move
        robot.backLeft_Drive.setTargetPosition(motorticks);
        robot.backLeft_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft_Drive.setPower(botSpeed * -botDirection);

        while(LO.opModeIsActive()&& robot.backLeft_Drive.isBusy()) {
             correction = checkOritation();
            //ever changing value to keep on tract
            flPower = botSpeed + correction * -botDirection;
            frPower = botSpeed - correction * botDirection  ;
            brPower = botSpeed - correction * botDirection  ;

            robot.backRight_Drive.setPower(brPower);
            robot.frontLeft_drive.setPower(flPower);
            robot.frontRight_Drive.setPower(frPower);
            //adding BLsetpower but may make infinte while busy loop
            LO.telemetry.addData("fr","%.2f",frPower);
            LO.telemetry.addData("fl","%.2f",flPower);
            LO.telemetry.addData("br","%.2f",brPower);
            LO.telemetry.addData("bl","%.2f",robot.backLeft_Drive.getPower());
            LO.telemetry.addData("tickcount",robot.backLeft_Drive.getCurrentPosition());
            LO.telemetry.update();
        }

        robot.backLeft_Drive.setPower(0);
        robot.frontRight_Drive.setPower(0);
        robot.backRight_Drive.setPower(0);
        robot.frontLeft_drive.setPower(0);

        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LO.sleep(500);
    }



    public void strafeMovement(int prewaitTime, double botSpeed, double botstrafeDirection, double runTime, double Distance) {
        double magicCorrectionNumber = 1.35, flPower, frPower, blPower, brPower, correction;
        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stepTime.reset();

        double DIstanceINCH = (COUNTS_PER_INCH) * (Distance) * magicCorrectionNumber;
        int motorticks = (int)Math.round(DIstanceINCH);
        LO.sleep(prewaitTime);
        robot.backLeft_Drive.setTargetPosition(motorticks);
        robot.backLeft_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft_Drive.setPower(botSpeed * -botstrafeDirection);

        while (LO.opModeIsActive() && robot.backLeft_Drive.isBusy()) {
            correction = checkOritation();
            flPower = botSpeed + correction * botstrafeDirection;
            blPower = botSpeed + correction * -botstrafeDirection;
            frPower = botSpeed - correction * botstrafeDirection  ;
            brPower = botSpeed - correction * -botstrafeDirection  ;
            robot.backRight_Drive.setPower(brPower);
            robot.frontLeft_drive.setPower(flPower);
            robot.frontRight_Drive.setPower(frPower);
            //also implemeted a test code for strafe correction
            LO.telemetry.addData("tickcount",robot.backLeft_Drive.getCurrentPosition());
            LO.telemetry.update();
        }

        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LO.sleep(500);
    }

    public void rotatebot(int degrees, double botSpeed, boolean stopAfterRotate){
        double flPower, frPower, blPower, brPower;
        resetAngle();
        // left and right are mirror oppoistes thanks to the wheels being mounted opposite ways
        if(degrees < 0){// right
            flPower = -botSpeed;
            blPower = -botSpeed;
            frPower = -botSpeed;
            brPower = -botSpeed;
        } else if (degrees >0){ //left
            flPower = botSpeed;
            blPower = botSpeed;
            frPower = botSpeed;
            brPower = botSpeed;
        } else return;
        robot.frontRight_Drive.setPower(frPower);
        robot.backRight_Drive.setPower(brPower );
        robot.backLeft_Drive.setPower(blPower);
        robot.frontLeft_drive.setPower(flPower);
        if(degrees < 0){
            while (LO.opModeIsActive() && getAngle() == 0) {
                LO.telemetry.addLine("rotationnotworking");
                LO.telemetry.update();
            }
         while (LO.opModeIsActive()&& getAngle() > degrees){
             LO. telemetry.addData("Global Angle:", globalAngle);
             LO.telemetry.addData("Correction:", correction);
             LO.telemetry.update();
         }
        }
        else while (LO.opModeIsActive()&&getAngle() <degrees){
            LO. telemetry.addData("Global Angle:", globalAngle);
            LO.telemetry.addData("Correction:", correction);
            LO.telemetry.update();
        }
        if(stopAfterRotate){
            stopBot(0,0,1);
        }
        LO.sleep(500);


        resetAngle();

    };

   // public double distanceDetect() {
        //detects distance and outputs
        // distance_Sensor
        //return robot.sensorRange.getDistance(DistanceUnit.MM);
   // }

    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }


    public double getAngle(){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    };

    public  double checkOritation(){
        double correction, angle, gain = 0.1;
        angle = getAngle();

        if (angle ==0)
        correction = 0;
        else correction = -angle;
        correction = correction * gain;

        return correction;
    }

    public void detectStones() {

    }

    public void calibrationIMU() {

    }

    public void colorSensor() {

    }

    public void stopBot(double stopPower, int prewaitTime, double runTime) {
        stepTime.reset();
        LO.sleep(prewaitTime);
        robot.frontRight_Drive.setPower(stopPower);
        robot.backRight_Drive.setPower(stopPower);
        robot.backLeft_Drive.setPower(stopPower);
        robot.frontLeft_drive.setPower(stopPower);
        while ((runTime < stepTime.milliseconds())) ;
        LO.telemetry.addData("Say", "Stopping bot");
        LO.telemetry.update();
    }




    //final colon for linearOpmode dont put functions past that
};