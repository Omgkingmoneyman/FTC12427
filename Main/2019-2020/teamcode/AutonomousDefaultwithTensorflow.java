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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//import org.firstinspires.ftc.teamcode.AutonomousFunctions;


@Autonomous(name="Autonomoustensorflow", group="Pushbot")

public class AutonomousDefaultwithTensorflow extends LinearOpMode {


    /* Declare OpMode members. */
    AutonomousFunctions af = new AutonomousFunctions();
    LinearOpMode LO = new LinearOpMode() {
        @Override
        public void runOpMode() throws InterruptedException {

        }
    };
    Hardware robot = new Hardware();

    //time and variables
    double globalAngle, correction;
    public ElapsedTime totalRuntime = new ElapsedTime();
    public ElapsedTime stepTime = new ElapsedTime();
    double botSpeed = 0.4;
    double correctionFactor = 1.3;
    public Integer skystonePosition = 0, stoneExist = 0, Stone, firstBoundary = 490, secondBoundary = 600;

    //tensorflow additions
    private static final String VUFORIA_KEY =
            "AZ9MU4z/////AAABmZGKr3YGfkXotPFo5uCpRKl8Tyqdl5ubYo/x2ndX0uxsfqJmJgkbpl3gaa4PBAUXhkfhfOH1jpZEEfBFTyrXrZFHBWzq8XqVdlAE9tGFBFh9AC7YTyavGGoMCiKmADgVsX2jAxoNUJJsGXjB+MqM5rwFsT/y8BGGbRXspm8JCZyYH+zo7tTj3lb91jALNYjZU1rkUxw+Hi1DDWUeLIepgn411Ch76mUYxAEAquWBqRVjYtqv1A0vbhediE9nLtynctGQfSw5dqDH8SX23LoYR5j9WofQVKUOc3xA/vCVMWqIKl58l0yLFolPYpNZNK+V2R9XSlEl3/VREL+/6gdxUOS0rdXxf50qt5OsnHoC/rJb ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    //values used for skystonedetection// skystonepostion is where its at and stoneexist is if its aculty there

    @Override

    // opmode
    public void runOpMode() {

        //tensorflow checks
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        //calibartion of imu goes here
        //must init our clases and hardware
        af.robot.init(hardwareMap);
        af.LO.init();
        robot.init(hardwareMap);

        //imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        BNO055IMU imu;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        //tensorflow
        if (tfod != null) {
            tfod.activate();
        }
        //imu check
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();


        //starting time
        totalRuntime.reset();
        //note bot needs to be in same starting point in same orination each time
        //since we refernce our functons from anthor class we need "af."
        while (opModeIsActive()|| LO.opModeIsActive() && (totalRuntime.seconds() < 30)) {

            //first step calibaratin of imu and starting of tensorflow
            //func that includes all 3 ways the bot can take off

            //steps for ending of automous here


            af.stepTime.reset();
            //af.moveforwardbackwards(0,botSpeed,1,2); example function on how to write a step
            af.moveforwardbackwards(0,botSpeed,-1,10);
            af.moveforwardbackwards(0,botSpeed,1,20);

            //telemetry.addData("distance", String.format("%.2f mm", distanceDetect()));
            //telemetry.update();


            af.stopBot(0,0,1);

        }

    }

        //autonomous functions

    public void SkystoneSteps() {
        if (skystonePosition == 1) {
            //first leg of program goes here and is left

            af.strafeMovement(0,botSpeed,1,0.8, correctionFactor);
            af.stopBot(0,0,500);
            af.moveforwardbackwards(0,botSpeed,1,.2);
            af.stopBot(0,0,500);
            af.robot.platformServo.setPosition(0);
        }
        if (skystonePosition == 2 || skystonePosition == 0) {
            //is middle
            af.strafeMovement(0,botSpeed,1,0.8,correctionFactor);
            af.stopBot(0,0,500);
            af.moveforwardbackwards(0,botSpeed,1,.2);
            af.stopBot(0,0,500);
            af.robot.platformServo.setPosition(0);
        }
        if (skystonePosition == 3 ) {
            //right side
            af.strafeMovement(0,botSpeed,1,0.8,correctionFactor);
            af.stopBot(0,0,500);
            af.moveforwardbackwards(0,botSpeed,1,.2);
            af.stopBot(0,0,500);
            af.robot.platformServo.setPosition(0);



        }
        LO.telemetry.update();
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //detection code for finding skystones and skyblocks
    public void skystoneDetect(double maxTime) {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }


            af.stepTime.reset();
            while (opModeIsActive() && (stepTime.seconds() < maxTime && skystonePosition == 0)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    //List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    List<Recognition> updatedRecognitions = tfod.getRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());


                    }
                    if(stoneExist == 0) {
                        //checking for blocks in list
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                stoneExist += 1;
                            }
                            if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                Stone += 1;
                            }
                        }
                    }
                    LO.telemetry.addData("# stones detected", Stone);
                    LO.telemetry.addData(" # skystoneExist", stoneExist);

                    LO.telemetry.update();
                    //firguring out where blocks are
                    if (stoneExist > 0){
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                LO.telemetry.addData("# Skystone Confidence",recognition.getConfidence());
                                LO.telemetry.addData("# Skystone Left Border", recognition.getLeft());
                                LO.telemetry.addData("# Skystone Right Border", recognition.getRight());
                                LO.telemetry.update();
                                if (recognition.getRight() <= firstBoundary && recognition.getLeft() <firstBoundary ) {
                                   LO.telemetry.addData("skystone Position =", "Left");
                                    skystonePosition = 1;
                                }
                                if (recognition.getLeft() <= firstBoundary && recognition.getRight() <= secondBoundary) {
                                    LO.telemetry.addData("skystone Position =", "middle");
                                    skystonePosition = 2;
                                }
                                if (recognition.getRight() >= secondBoundary) {
                                    LO.telemetry.addData("skystone Position =", "Right");
                                    skystonePosition = 3;
                                }

                            }
                        }
                    }
                    LO.telemetry.update();

                }
                if (tfod != null) {
                    tfod.shutdown();
                }
            }


        }

    }

    ;
//final colon nothing before this
}