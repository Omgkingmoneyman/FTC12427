
 package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareFile;

import java.util.List;

//import org.firstinspires.ftc.teamcode.AutonomousFunctions
@Autonomous(name="testencoder", group="Pushbot")

public class testautonomousencocer extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareFile robot = new HardwareFile();   // Use a Pushbot's hardware
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation angles;
    Acceleration gravity;

    //public AutonomousFunctions af = new  AutonomousFunctions();
    double globalAngle,  correction;
    public Integer skystonePosition = 0, stoneExist = 0, Stone, firstBoundary = 490, secondBoundary = 600;
    public ElapsedTime totalRuntime = new ElapsedTime();
    public ElapsedTime stepTime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV ) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final String VUFORIA_KEY =
            "AZ9MU4z/////AAABmZGKr3YGfkXotPFo5uCpRKl8Tyqdl5ubYo/x2ndX0uxsfqJmJgkbpl3gaa4PBAUXhkfhfOH1jpZEEfBFTyrXrZFHBWzq8XqVdlAE9tGFBFh9AC7YTyavGGoMCiKmADgVsX2jAxoNUJJsGXjB+MqM5rwFsT/y8BGGbRXspm8JCZyYH+zo7tTj3lb91jALNYjZU1rkUxw+Hi1DDWUeLIepgn411Ch76mUYxAEAquWBqRVjYtqv1A0vbhediE9nLtynctGQfSw5dqDH8SX23LoYR5j9WofQVKUOc3xA/vCVMWqIKl58l0yLFolPYpNZNK+V2R9XSlEl3/VREL+/6gdxUOS0rdXxf50qt5OsnHoC/rJb ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    //values used for skystonedetection// skystonepostion is where its at and stoneexist is if its aculty there

    int motorticks = 0;
    double botSpeed  = 0.35; //botSpeedLB =botSpeed * 1,botSpeedLF = botSpeed * 1,botspeedRF = botSpeed * 1, botspeedRB = botSpeed * 1;
    double circumfernce = 9.42;
    double flPower, frPower, blPower, brPower;
    double encoderSpeed =1.2;



    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        //calibartion of imu goes here
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //robot.backLeft_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
        //robot.backRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.frontLeft_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        //robot.frontRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);


        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //tensorflow
        if (tfod != null) {
            tfod.activate();
        }
        //imu check
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();


        waitForStart();
        //starting time
        totalRuntime.reset();
        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stepTime.reset();
       //code here
        moveforwardbackwards(0,botSpeed,"forward,",12,true);

        stopBot(0,0,1);
        strafeMovement(0,botSpeed,"right",12,true,encoderSpeed);





        stopBot(0,0,1);
        stop();
    }

    public void moveforwardbackwards(int prewaitTime, double botSpeed, String botDirection , double Distance, boolean Correctionon) {
        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);;
        double flPower, frPower, blPower, brPower, correction;
        if (botDirection == "forward" ) {
            robot.backLeft_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.backRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontLeft_drive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        } else
        {robot.backLeft_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backRight_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontLeft_drive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontRight_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        stepTime.reset();
        //finding the amount of ticks needed to move based off what is inputed
        double DistanceINCH = (COUNTS_PER_INCH) * (Distance);
        int motorticks = (int)Math.round(DistanceINCH);
        //my variables
        blPower = botSpeed; brPower = botSpeed; flPower = botSpeed; frPower = botSpeed ;
        sleep(prewaitTime);
        //inputing amount of ticks needed to move then setting power to move
        robot.backLeft_Drive.setTargetPosition(motorticks);
        robot.backLeft_Drive.setPower(blPower);
        robot.backRight_Drive.setPower(brPower);
        robot.frontLeft_drive.setPower(flPower);
        robot.frontRight_Drive.setPower(frPower);
        robot.backLeft_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()&& robot.backLeft_Drive.isBusy()) {
            blPower=botSpeed; brPower = botSpeed; flPower = botSpeed; frPower = botSpeed ;
            correction = checkOritation();
            //ever changing value to keep on tract
            //other wheels will correct based on what blmotor current movement is
            if (botDirection =="forward"&& Correctionon) {
                flPower = botSpeed - correction;
                blPower = botSpeed - correction;
                frPower = botSpeed + correction;
                brPower = botSpeed + correction;
            } else if(botDirection == "backward" && Correctionon) {
                flPower = botSpeed + correction;
                blPower = botSpeed + correction;
                frPower = botSpeed - correction;
                brPower = botSpeed - correction;
            }
            robot.backRight_Drive.setPower(brPower);
            robot.frontLeft_drive.setPower(flPower);
            robot.frontRight_Drive.setPower(frPower);
            robot.backLeft_Drive.setPower(blPower);
            //adding BLsetpower but may make infinte while busy loop
            telemetry.addData("tickcount",robot.backLeft_Drive.getCurrentPosition());
            telemetry.update();
        }

        robot.backLeft_Drive.setPower(0);
        robot.frontRight_Drive.setPower(0);
        robot.backRight_Drive.setPower(0);
        robot.frontLeft_drive.setPower(0);

        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        sleep(500);
    }

    public void strafeMovement(int prewaitTime, double botSpeed, String botstrafeDirection, double Distance, boolean Correctionon,double encoderSpeed) {
        double magicCorrectionNumber = 1.00, flPower, frPower, blPower, brPower, correction;
        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stepTime.reset();
        //values are changed for strafing and not forward or back
        //determines what direction it goes
        if (botstrafeDirection == "left" ) {
            robot.backLeft_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.backRight_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontLeft_drive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (botstrafeDirection =="right") {
            robot.backLeft_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontLeft_drive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontRight_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        double DistanceINCH = (COUNTS_PER_INCH) * (Distance) * magicCorrectionNumber;
        int motorticks = (int)Math.round(DistanceINCH);
        blPower=botSpeed; brPower = botSpeed; flPower = botSpeed; frPower = botSpeed ;

        sleep(prewaitTime);
        robot.backLeft_Drive.setTargetPosition(motorticks);
        robot.backLeft_Drive.setPower(blPower);
        robot.backRight_Drive.setPower(brPower);
        robot.frontLeft_drive.setPower(flPower);
        robot.frontRight_Drive.setPower(frPower);
        robot.backLeft_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.backLeft_Drive.isBusy()) {
            blPower=botSpeed * encoderSpeed;
            brPower = botSpeed; flPower = botSpeed; frPower = botSpeed ;
            correction = checkOritation();
            if(botstrafeDirection =="left" && Correctionon) {
                flPower = botSpeed + correction;
                frPower = botSpeed - correction;
                brPower = botSpeed - correction;
                blPower = botSpeed + correction;
            } else if (botstrafeDirection =="right" && Correctionon){
                flPower = botSpeed - correction;
                frPower = botSpeed + correction;
                brPower = botSpeed + correction;
                blPower = botSpeed - correction;
            }
            robot.backLeft_Drive.setPower(blPower);
            robot.backRight_Drive.setPower(brPower);
            robot.frontLeft_drive.setPower(flPower);
            robot.frontRight_Drive.setPower(frPower);
            //values are based for blmotor
            //also implemeted a test code for strafe correction
            telemetry.addData("tickcount",robot.backLeft_Drive.getCurrentPosition());
            telemetry.addData("correction",correction);
            telemetry.update();
        }

        stopBot(0.0,0,.5);

        sleep(500);
    }

    public void rotatebot(int degrees, double botSpeed, boolean stopAfterRotate){
        double flPower, frPower, blPower, brPower;
        resetAngle();
        // left and right are mirror oppoistes thanks to the wheels being mounted opposite ways
        if(degrees < 0){// right
            //going to test without the theses fucntions and see if -botspeed and botspeed still work
            //robot.backLeft_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
            //robot.backRight_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
            //robot.frontLeft_drive.setDirection(DcMotorSimple.Direction.FORWARD);
            //robot.frontRight_Drive.setDirection(DcMotorSimple.Direction.FORWARD);
            flPower = -botSpeed;
            blPower = -botSpeed;
            frPower = -botSpeed;
            brPower = -botSpeed;
        } else if (degrees >0){
            // robot.backLeft_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            //robot.backRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            //robot.frontLeft_drive.setDirection(DcMotorSimple.Direction.REVERSE);
            //robot.frontRight_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
            //left
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
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addLine("rotationnotworking");
                telemetry.update();
            }
            while (opModeIsActive()&& getAngle() > degrees){
                telemetry.addData("Global Angle:", globalAngle);
                telemetry.addData("Correction:", correction);
                telemetry.update();
            }
        }
        else while (opModeIsActive()&&getAngle() <degrees){
            telemetry.addData("Global Angle:", globalAngle);
            telemetry.addData("Correction:", correction);
            telemetry.update();
        }
        if(stopAfterRotate){
            stopBot(0.0,0,1);
        }
        sleep(500);


        resetAngle();

    };



    public void calibrationIMU() {

    }

    public void colorSensor() {

    }

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
        double correction, angle, gain = 0.03;
        angle = getAngle();

        if (angle ==0)
            correction = 0;
        else correction = -angle;
        correction = correction * gain;

        return correction;
    }

    public void stopBot(double stopPower, int prewaitTime, double runTime) {
        stepTime.reset();
        sleep(prewaitTime);
        robot.frontRight_Drive.setPower(stopPower);
        robot.backRight_Drive.setPower(stopPower);
        robot.backLeft_Drive.setPower(stopPower);
        robot.frontLeft_drive.setPower(stopPower);
        while ((runTime < stepTime.seconds())) ;
        telemetry.addData("Say", "Stopping bot");
        telemetry.update();
        robot.backLeft_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void stopencoderBot (int prewaitTime, double runTime){

        stepTime.reset();
        sleep(prewaitTime);



    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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


    //final colon for linearOpmode dont put functions past that
};