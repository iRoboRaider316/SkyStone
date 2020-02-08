package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class SteveBase {

    OpMode opMode;
    DcMotor motorDriveLF;
    DcMotor motorDriveLB;
    DcMotor motorDriveRF;
    DcMotor motorDriveRB;
    DcMotor motorCollectionL;
    DcMotor motorCollectionR;
    DcMotor motorLiftL;
    DcMotor motorLiftR;
    Servo servoFoundationL;
    Servo servoFoundationR;
    Servo servoGrabber;
    CRServo servoCapstone;
    CRServo servoFourbarArmL;
    CRServo servoFourbarArmR;
    CRServo servoPark;
    DistanceSensor sensorColor;
    ColorSensor sensorSkystones;
    DigitalChannel foundationTouchR;
    DigitalChannel foundationTouchL;

    BNO055IMU imu;                  // IMU Gyro itself
    Orientation angles;             // IMU Gyro's Orienting
    File headingFile = AppUtil.getInstance().getSettingsFile("headingFile");

    ElapsedTime timerOpMode;
    ElapsedTime timerTravel;
    public ElapsedTime buttonPress = new ElapsedTime();
    public ElapsedTime transferTimer = new ElapsedTime();

    double angleTest[] = new double[10];
    int count = 0;
    double sum;
    double correct;
    double attemptsToGrabFoundation = 0;

    String allianceColor;
    boolean scoreSkyStones = false;
    String parkingPreference;
    boolean pushingFoundation = false;

    String skystonePosition = "";
    String[] skystonePositions = {"LEFT", "CENTER", "RIGHT"};

    double leftWheelTickDelta = 0;
    double rightWheelTickDelta = 0;
    double strafeWheelTickDelta = 0;
    double leftWheelTickDeltaPrevious = 0;
    double rightWheelTickDeltaPrevious = 0;
    double strafeWheelTickDeltaPrevious = 0;
    double xPosition = 0;
    double yPosition = 0;
    double initialHeading = 90;
    double robotHeading = 0;
    double robotHeadingDelta = 0;
    double robotHeadingPrevious = 0;
    double lockedHeading;

    boolean driveCollect = false;
    boolean autoTransfer = true;
    int hopperState = 0;

    double ENCODER_CPR = 360;
    double WHEEL_CURCUMFERENCE = 2.28;
    double COUNTS_PER_INCH = ENCODER_CPR / WHEEL_CURCUMFERENCE;
    double STARTING_HEADING = 0;
    double STRAFE_WHEEL_OFFSET_CONSTANT = 354; // Unit = DISTANCE / ANGLE
    double TARGET_POSITION_ACCURACY_IN_INCHES = 0.5;
    double TARGET_HEADING_ACCURACY_IN_DEGREES = 2;
    double SKYSTONE_DETECTION_THRESHHOLD = 200;

    private enum TransferState {
        IDLE, LOWERING_LINKAGE, GRABBING, GRABBED;
    }
    TransferState transfer = TransferState.IDLE;

    public SteveBase(OpMode theOpMode) {
        opMode = theOpMode;

        // INITIALIZE MOTORS AND SERVOS
        opMode.telemetry.addLine("Initalizing output devices (motors, servos)...");
        opMode.telemetry.update();

        motorDriveLF = opMode.hardwareMap.dcMotor.get("motorDriveLF");
        motorDriveLB = opMode.hardwareMap.dcMotor.get("motorDriveLB");
        motorDriveRF = opMode.hardwareMap.dcMotor.get("motorDriveRF");
        motorDriveRB = opMode.hardwareMap.dcMotor.get("motorDriveRB");

        motorDriveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveLF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveLB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveRB.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoders();
        runWithoutEncoders();

        motorCollectionL = opMode.hardwareMap.dcMotor.get("motorCollectionL");
        motorCollectionR = opMode.hardwareMap.dcMotor.get("motorCollectionR");
        motorLiftL = opMode.hardwareMap.dcMotor.get("motorLiftL");
        motorLiftR = opMode.hardwareMap.dcMotor.get("motorLiftR");

        servoFoundationL = opMode.hardwareMap.servo.get("servoFoundationL");
        servoFoundationR = opMode.hardwareMap.servo.get("servoFoundationR");
        servoFoundationL.setPosition(1);
        servoFoundationR.setPosition(0);
        
        servoGrabber = opMode.hardwareMap.servo.get("servoGrabber");
        servoGrabber.setPosition(1);

        servoCapstone = opMode.hardwareMap.crservo.get("servoCapstone");

        servoFourbarArmL = opMode.hardwareMap.crservo.get("servoFourbarArmL");
        servoFourbarArmR = opMode.hardwareMap.crservo.get("servoFourbarArmR");

        servoPark = opMode.hardwareMap.crservo.get("servoPark");

        // INITIALIZE SENSORS
        opMode.telemetry.addLine("Initalizing input devices (sensors)...");
        opMode.telemetry.update();

        sensorColor = opMode.hardwareMap.get(DistanceSensor.class, "sensorColor");
        sensorSkystones = opMode.hardwareMap.get(ColorSensor.class, "sensorSkystones");

        foundationTouchL = opMode.hardwareMap.get(DigitalChannel.class, "foundationTouchL");
        foundationTouchR = opMode.hardwareMap.get(DigitalChannel.class, "foundationTouchR");

        foundationTouchL.setMode(DigitalChannel.Mode.INPUT);
        foundationTouchR.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);

        timerOpMode = new ElapsedTime();
        timerTravel = new ElapsedTime();

        opMode.telemetry.addLine("Initialization Succeeded!");
        opMode.telemetry.update();

    }

    public void resetEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runWithEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDrivePower(double motorPower) {
        setDrivePowerSides(motorPower, motorPower);
    }

    public void setDrivePowerSides(double motorPowerL, double motorPowerR) {
        setDrivePowerMotors(motorPowerL, motorPowerL, motorPowerR, motorPowerR);
    }

    public void setDrivePowerMotors(double motorPowerLF, double motorPowerLB, double motorPowerRF, double motorPowerRB) {
        motorDriveLF.setPower(motorPowerLF);
        motorDriveLB.setPower(motorPowerLB);
        motorDriveRF.setPower(motorPowerRF);
        motorDriveRB.setPower(motorPowerRB);
    }

    public boolean isFoundationGrabbed() {
        // Since getState() returns false for pressed and true for not pressed, we had to apply some logic inverters to return results that make sense.
        return (!foundationTouchL.getState() || !foundationTouchR.getState());
    }

    /* =======================AUTONOMOUS EXCLUSIVE METHODS========================= */
    public void sleep(long ms) {
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }

    boolean isInitialized() {
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }

    public void selection() {
        // What alliance color are we? (By the way, this is a do-while loop. It always runs at least once because of how it's set up)
        opMode.telemetry.addLine("Awaiting Autonomous Selection...");
        opMode.telemetry.addData("For blue alliance, press", "X");
        opMode.telemetry.addData("For red alliance, press", "B");
        opMode.telemetry.update();

        do {
            sleep(50);
            if(opMode.gamepad1.x) allianceColor = "BLUE";
            if(opMode.gamepad1.b) allianceColor = "RED";
        } while(!opMode.gamepad1.x && !opMode.gamepad1.b && isInitialized());

        // Parking Preference?
        opMode.telemetry.addLine("Awaiting Autonomous Selection...");
        opMode.telemetry.addData("To park inside, press", "Y");
        opMode.telemetry.addData("To park outside, press", "A");
        opMode.telemetry.update();

        do {
            sleep(50);
            if(opMode.gamepad1.y) parkingPreference = "INSIDE";
            if(opMode.gamepad1.a) parkingPreference = "OUTSIDE";
        } while(!opMode.gamepad1.y && !opMode.gamepad1.a && isInitialized());

        // Score SkyStones?
        opMode.telemetry.addLine("Awaiting Autonomous Selection...");
        opMode.telemetry.addData("To score the SkyStones, press:", "X");
        opMode.telemetry.addData("To move the foundation in auto, press", "B");
        opMode.telemetry.addData("To just park, press", "START");
        opMode.telemetry.update();

        do {
            sleep(50);
            if(opMode.gamepad1.x) scoreSkyStones = true;
            if(opMode.gamepad1.b) pushingFoundation = true;
        } while(!opMode.gamepad1.x && !opMode.gamepad1.b && !opMode.gamepad1.start && isInitialized());

        opMode.telemetry.addData("All Set! Just press PLAY when ready!", "D");
        opMode.telemetry.addData("Current Alliance", allianceColor);
        opMode.telemetry.addData("Parking Preference", parkingPreference);
        opMode.telemetry.addData("Score SkyStone in Auto?", scoreSkyStones);
        opMode.telemetry.addData("Move Foundation in Auto?", pushingFoundation);
        opMode.telemetry.update();

        /*if(scoreSkyStones) {
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection   = CAMERA_CHOICE;
            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
            // Load the data sets for the trackable objects.
            targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
            allTrackables = new ArrayList<>();
            allTrackables.addAll(targetsSkyStone);
            *//**
             * If you are standing in the Red Alliance Station looking towards the center of the field,
             *     - The X axis runs from your left to the right. (positive from the center to the right)
             *     - The Y axis runs from the Red Alliance Station towards the other side of the field
             *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
             *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
             *//*
            // Set the position of the Stone Target.  Assume it's at the field origin. Rotated it to face forward, and raised it to sit on the ground correctly.
            stoneTarget.setLocation(OpenGLMatrix
                    .translation(0, 0, stoneZ)
                    .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90, 0, -90)));
            // Info:  The coordinate frame for the robot looks the same as the field.
            // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
            // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
            //
            // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
            // pointing to the LEFT side of the Robot.
            // We need to rotate the camera around it's long axis to bring the correct camera forward.
            if (CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }
            // Rotate the phone vertical about the X axis if it's in portrait mode
            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90 ;
            }
            // Next, translate the camera lens to where it is on the robot.
            final float CAMERA_FORWARD_DISPLACEMENT  = 6f * mmPerInch;   //sets the phone at the exact front of the robot (code correction for x axis telemetry consistently being off by an inch)
            final float CAMERA_VERTICAL_DISPLACEMENT = 5f * mmPerInch;   // Camera is 4 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT     = 8f * mmPerInch;   // SIDE TO SIDE
            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
            *//**  Let all the trackable listeners know where the phone is.  *//*
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }
        }*/
    }

    public void updateOdometry(DcMotor encoderLeft, DcMotor encoderRight, DcMotor encoderStrafe) {
        // STEP 1: Calculate the Delta (change) of the ticks and robot heading since last update
        leftWheelTickDelta = -(encoderLeft.getCurrentPosition() - leftWheelTickDeltaPrevious);
        rightWheelTickDelta = (encoderRight.getCurrentPosition() - rightWheelTickDeltaPrevious);
        strafeWheelTickDelta = -(encoderStrafe.getCurrentPosition() - strafeWheelTickDeltaPrevious);

        leftWheelTickDeltaPrevious = encoderLeft.getCurrentPosition();
        rightWheelTickDeltaPrevious = encoderRight.getCurrentPosition();
        strafeWheelTickDeltaPrevious = encoderStrafe.getCurrentPosition();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotHeading = angles.firstAngle + initialHeading;
        robotHeadingDelta = robotHeading - robotHeadingPrevious;

        robotHeadingPrevious = robotHeading;

        // STEP 2: Refine the changes to get the vertical and horizontal changes in inches
        double leftDistance = leftWheelTickDelta / COUNTS_PER_INCH;
        double rightDistance = rightWheelTickDelta / COUNTS_PER_INCH;
        double verticalDistance = (leftDistance + rightDistance) / 2;
        double horizontalDistance = (strafeWheelTickDelta - (Math.toRadians(robotHeadingDelta) / STRAFE_WHEEL_OFFSET_CONSTANT)) / COUNTS_PER_INCH;

        // STEP 3: Apply Trigonometry to the vertical and horizontal changes and update our position
        // Position Update for the vertical encoder wheels
        xPosition += verticalDistance * Math.cos(Math.toRadians(robotHeading));
        yPosition += verticalDistance * Math.sin(Math.toRadians(robotHeading));
        // Position Update for the horizontal encoder wheel
        xPosition += horizontalDistance * Math.sin(Math.toRadians(robotHeading));
        yPosition -= horizontalDistance * Math.cos(Math.toRadians(robotHeading));

        double robotSpeedInFPS = Math.hypot(horizontalDistance, verticalDistance) / (timerTravel.seconds() * 12);
        opMode.telemetry.addData("Robot Speed in Feet per Second", robotSpeedInFPS);
    }

    void travelToPosition(double xTarget, double yTarget, double offsetHeading, double... powerMinAndMax) {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        lockedHeading = robotHeading + offsetHeading;
        double headingError = lockedHeading - robotHeading;
        double xDelta = -xTarget + xPosition;
        double yDelta = yTarget - yPosition;
        double headingBound = lockedHeading + 180;

        while(((Math.abs(xDelta) > TARGET_POSITION_ACCURACY_IN_INCHES ||
              Math.abs(yDelta) > TARGET_POSITION_ACCURACY_IN_INCHES) &&
              ((LinearOpMode)opMode).opModeIsActive())) {
            updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
            xDelta = -xTarget + xPosition;
            yDelta = yTarget - yPosition;
            headingError = lockedHeading - robotHeading;
            headingError += headingError > headingBound ? -360 :
                            headingError < headingBound - 360 ? 360 : 0;

            double desiredSpeedLF = 0;
            double desiredSpeedLB = 0;
            double desiredSpeedRF = 0;
            double desiredSpeedRB = 0;

            if(Math.abs(xDelta) > TARGET_POSITION_ACCURACY_IN_INCHES ||
               Math.abs(yDelta) > TARGET_POSITION_ACCURACY_IN_INCHES) {
                // Determines wheel power for each motor based on our base motor power and target X, Y, and Theta values
                double angleDirection = Math.atan2(yDelta, xDelta) + Math.toRadians(robotHeading) - Math.toRadians(initialHeading);
                double drivingPower = 0.55;
                double wheelTrajectory = angleDirection - (Math.PI/4);
                double powerMod = Math.abs(Math.hypot(xDelta, yDelta)) > 5 ? 1 :
                                  powerMinAndMax.length == 1 ? powerMinAndMax[0] : 0.45;

                desiredSpeedLF -= (drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2) * powerMod;
                desiredSpeedLB -= (drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2) * powerMod;
                desiredSpeedRF += (drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2) * powerMod;
                desiredSpeedRB += (drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2) * powerMod;
            }
            // Correct for robot drifting
            if(Math.abs(headingError) > TARGET_HEADING_ACCURACY_IN_DEGREES) {
                double turnMod = Range.clip(Math.toRadians(headingError * 1.5), -0.5, 0.5);
                desiredSpeedLF = Range.clip(desiredSpeedLF - turnMod, -1, 1);
                desiredSpeedLB = Range.clip(desiredSpeedLB - turnMod, -1, 1);
                desiredSpeedRF = Range.clip(desiredSpeedRF - turnMod, -1, 1);
                desiredSpeedRB = Range.clip(desiredSpeedRB - turnMod, -1, 1);
            }

            setDrivePowerMotors(desiredSpeedLF, desiredSpeedLB, desiredSpeedRF, desiredSpeedRB);
        }
        setDrivePower(0);
        imuTurn(headingError);
    }

    /** Just an angle converter to use in case we ever need to turn to an absolute heading, instead
     * of a heading relative to the robot.*/
    public double absoluteHeading(double target) {
        target += 180;
        target += target > 360 ? -360 :
                target <   0 ?  360 : 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle + 180;
        double degreesToTurn = target - currentHeading;
        degreesToTurn += degreesToTurn > 180 ? -360 :
                         degreesToTurn < -180 ? 360 : 0;
        return degreesToTurn;
    }

    /** The IMU Turn we used last year. Not as complicated as the encoder drive, yet more
     * complicated at the same time. Enjoy.*/
    private void imuTurn(double degreesToTurn) {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        double currentHeading = angles.firstAngle + 180;
        double targetHeading = degreesToTurn + currentHeading;
        targetHeading += targetHeading > 360 ? -360 :
                         targetHeading < 0 ? 360 : 0;

        timerTravel.reset();
        while (Math.abs(degreesToTurn) > 2 && ((LinearOpMode)opMode).opModeIsActive() && timerTravel.seconds() <= 2) {
            updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
            currentHeading = angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            /*double POWER_CAP = 45;      // Once we hit this power, we should slow down.
            double DEGREES_TO_POWER = Range.clip(Math.pow(degreesToTurn / POWER_CAP, 1/3), -1, 1);
            double TURN_POWER = DEGREES_TO_POWER * .7;*/
            double TURN_POWER = Range.clip(Math.signum(degreesToTurn) * (0.2 + (Math.abs(degreesToTurn) / 270)), -1, 1);
            setDrivePowerSides(-TURN_POWER, -TURN_POWER);

            opMode.telemetry.addData("DegreesToTurn", degreesToTurn);
            opMode.telemetry.addData("Target", targetHeading);
            opMode.telemetry.addData("Current", currentHeading);
            opMode.telemetry.addData("position LF", motorDriveLF.getCurrentPosition());
            opMode.telemetry.addData("position LB", motorDriveLB.getCurrentPosition());
            opMode.telemetry.addData("position RF", motorDriveRF.getCurrentPosition());
            opMode.telemetry.addData("position RB", motorDriveRB.getCurrentPosition());
            opMode.telemetry.update();
        }
        setDrivePower(0);
        sleep(100);
        // Using the power of recursion, we can keep calling this method to account for errors in turning.
        if(Math.abs(degreesToTurn) > 2 && ((LinearOpMode)opMode).opModeIsActive()) {
            imuTurn(degreesToTurn);
        }
    }

    public void grabFoundation() {
        if(allianceColor.equals("RED")) {
            travelToPosition(13, 26, 0);
            travelToPosition(13, 31, 0);
            while(((LinearOpMode)opMode).opModeIsActive() && attemptsToGrabFoundation < 6 && timerOpMode.seconds() < 29) {
                servoFoundationL.setPosition(0);
                servoFoundationR.setPosition(1);
                attemptsToGrabFoundation++;
                sleep(1000);
                travelToPosition(13, 31 + (attemptsToGrabFoundation * 2), 0);
                if(isFoundationGrabbed()) {
                    break;
                }
                else if(((LinearOpMode)opMode).opModeIsActive()){
                    servoFoundationL.setPosition(1);
                    servoFoundationR.setPosition(0);
                    sleep(600);
                    attemptsToGrabFoundation++;
                    travelToPosition(13, 31 + (attemptsToGrabFoundation * 2), 0);
                }
            }
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
            sleep(800);
            travelToPosition(15, yPosition, 0);
            travelToPosition(15, 31, 0, 0.35);
        } else if(allianceColor.equals("BLUE")) {
            travelToPosition(-13, 26, 0);
            travelToPosition(-13, 31, 0);
            while(((LinearOpMode)opMode).opModeIsActive() && attemptsToGrabFoundation < 6 && timerOpMode.seconds() < 29) {
                servoFoundationL.setPosition(0);
                servoFoundationR.setPosition(1);
                sleep(1000);
                attemptsToGrabFoundation++;
                travelToPosition(-13, 31 + (attemptsToGrabFoundation * 2), 0);
                if(isFoundationGrabbed()) {
                    break;
                }
                else if(((LinearOpMode)opMode).opModeIsActive()){
                    servoFoundationL.setPosition(1);
                    servoFoundationR.setPosition(0);
                    sleep(600);
                    attemptsToGrabFoundation++;
                    travelToPosition(-13, 31 + (attemptsToGrabFoundation * 2), 0);
                }
            }
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
            sleep(800);
            travelToPosition(-15, yPosition, 0);
            travelToPosition(-15, 31, 0, 0.35);
        }
    }

    public void turnAndScoreFoundation(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(allianceColor.equals("RED")){
            while (angles.firstAngle > -85 && ((LinearOpMode)opMode).opModeIsActive() && timerOpMode.seconds() < 29){
                updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
                setDrivePowerSides(1, -0.3);
            }
            travelToPosition(3, 5, 0, 0.4);
        } else if(allianceColor.equals("BLUE")){
            while (angles.firstAngle < 85 && ((LinearOpMode)opMode).opModeIsActive() && timerOpMode.seconds() < 29){
                updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
                setDrivePowerSides(0.3, -1);
            }
            travelToPosition(-4, 5, 0, 0.4);
        }
        servoFoundationL.setPosition(1);
        servoFoundationR.setPosition(0);
        sleep(600);
    }

    void lookForSkystones() {
        if(allianceColor.equals("RED")) {
            for(int position = 0; position < 3; position++) { // Going from left to right
                travelToPosition(-28.5, -4 + (8 * position), 0);
                if(sensorSkystones.alpha() < SKYSTONE_DETECTION_THRESHHOLD) {
                    skystonePosition = skystonePositions[position];
                    return;
                }
            }
            // If Skystone isn't found, just grab the left stone because it's the easiest to intake.
            skystonePosition = "LEFT";
        } else if(allianceColor.equals("BLUE")) {
            for(int position = 2; position > -1; position--) { // Going from right to left
                travelToPosition(28.5, 4 - (8 * position), 0);
                if(sensorSkystones.alpha() < SKYSTONE_DETECTION_THRESHHOLD) {
                    skystonePosition = skystonePositions[position];
                    return;
                }
            }
            // If Skystone isn't found, just grab the right stone because it's the easiest to intake.
            skystonePosition = "RIGHT";
        }
    }

    public void collectAndScoreSkystones(){
        lookForSkystones();
        if(allianceColor.equals("RED")) {
            if(skystonePosition.equals("LEFT")) {
                double stonePosX = -39.5;
                double stonePosY = 8;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;

                travelToPosition(xPosition, stonePosY, 0);
                travelToPosition(stonePosX, stonePosY, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 4, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 16, 180);
                travelToPosition(skybridgePassingX, 51 ,0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
                travelToPosition(skybridgePassingX, 16, 0);
                travelToPosition(stonePosX + 12, stonePosY - 24, -180);
                travelToPosition(stonePosX, stonePosY - 24, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 28, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 16, 180);
                travelToPosition(skybridgePassingX, 51 ,0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
            } else if(skystonePosition.equals("CENTER")) {
                double stonePosX = -39.5;
                double stonePosY = 16;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;

                travelToPosition(xPosition, stonePosY, 0);
                travelToPosition(stonePosX, stonePosY, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 4, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 16, 180);
                travelToPosition(skybridgePassingX, 51 ,0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
                travelToPosition(skybridgePassingX, 16, 0);
                travelToPosition(stonePosX + 12, stonePosY - 24, -180);
                travelToPosition(stonePosX, stonePosY - 24, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 28, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 16, 180);
                travelToPosition(skybridgePassingX, 51 ,0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
            } else if(skystonePosition.equals("RIGHT")) {
                double stonePosX = -31.5;
                double stonePosY = 20;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;

                travelToPosition(xPosition, stonePosY, -45);
                travelToPosition(stonePosX, stonePosY, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX - 6, stonePosY - 6, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 16, 225);
                travelToPosition(skybridgePassingX, 51, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
                travelToPosition(skybridgePassingX, 16, 0);
                travelToPosition(stonePosX + 12, stonePosY - 24, -225);
                travelToPosition(stonePosX, stonePosY - 24, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX - 6, stonePosY - 30, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 16, 225);
                travelToPosition(skybridgePassingX, 51, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
            }
        } else if(allianceColor.equals("BLUE")) {
            if (skystonePosition.equals("LEFT")) {
                double stonePosX = 39.5;
                double stonePosY = 0;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? 26.5 : 5;

                travelToPosition(stonePosX, stonePosY, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 4, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 10, -180);
                travelToPosition(skybridgePassingX, 45, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
                travelToPosition(skybridgePassingX, 10, 0);
                travelToPosition(stonePosX - 10, stonePosY - 24, 180);
                travelToPosition(stonePosX, stonePosY - 24, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 28, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 10, -180);
                travelToPosition(skybridgePassingX, 45, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
            } else if (skystonePosition.equals("CENTER")) {
                double stonePosX = 39.5;
                double stonePosY = 8;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? 26.5 : 5;

                travelToPosition(xPosition, stonePosY, 0);
                travelToPosition(stonePosX, stonePosY, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 4, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 10, -180);
                travelToPosition(skybridgePassingX, 45, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
                travelToPosition(skybridgePassingX, 10, 0);
                travelToPosition(stonePosX - 10, stonePosY - 24, 180);
                travelToPosition(stonePosX, stonePosY - 24, 0);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 28, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 10, -180);
                travelToPosition(skybridgePassingX, 45, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
            } else if (skystonePosition.equals("RIGHT")) {
                double stonePosX = 31.5;
                double stonePosY = 12;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? 26.5 : 5;

                travelToPosition(xPosition, stonePosY, 0);
                travelToPosition(stonePosX, stonePosY, 45);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX + 4, stonePosY - 4, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 10, -225);
                travelToPosition(skybridgePassingX, 45, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
                travelToPosition(skybridgePassingX, 10, 0);
                travelToPosition(stonePosX, stonePosY - 24, 225);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX + 4, stonePosY - 28, 0);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                travelToPosition(skybridgePassingX, 10, -225);
                travelToPosition(skybridgePassingX, 45, 0);
                motorCollectionL.setPower(-0.5);
                motorCollectionR.setPower(0.5);
            }
        }
    }

    public void driveToSkybridge() {
        if(timerOpMode.seconds() < 29) {
            if(scoreSkyStones) {
                if (allianceColor.equals("RED")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(26.5, -35, 0);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(5, -35, 0);
                    }
                } else if (allianceColor.equals("BLUE")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(30, 35, 0);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(5, 35, 0);
                    }
                }
            } else {
                if (allianceColor.equals("RED")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(-20, 31, 0);
                        travelToPosition(-40, 31, 0);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(-40, 6, 0);
                    }
                } else if (allianceColor.equals("BLUE")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(18, 31, 0);
                        travelToPosition(35, 31, 0);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(35, 5, 0);
                    }
                }
            }
        }


    }

    public void waitForEnd(){
        while (timerOpMode.seconds() <29){
        }
    }

    public void storeHeading(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        ReadWriteFile.writeFile(headingFile, String.valueOf(-angles.firstAngle));
        opMode.telemetry.addData("headingFile", "" + ReadWriteFile.readFile(headingFile)); //Store robot heading to phone
        opMode.telemetry.update();
    }

    /* =======================TELEOP METHODS========================= */
    public void retrieveHeading() {
        try {
            STARTING_HEADING = Double.parseDouble(ReadWriteFile.readFile(headingFile));
        } catch(Exception exc) {
            opMode.telemetry.addData("ERROR", "Couldn't read headingFile; will set startingHeading to 0.");
            opMode.telemetry.update();
            STARTING_HEADING = 0;
        }
    }

    public void resetHeading(){ //Resets the imu heading by adding/subtracting from itself
        if (opMode.gamepad1.b){ // In case somethine goes wrong, driver can reposition the robot and reset the heading during teleop
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (angles.firstAngle > 0){
                angles.firstAngle -= 2 * angles.firstAngle;
            }
            else {
                angles.firstAngle += 2 * angles.firstAngle;
            }
            STARTING_HEADING = angles.firstAngle;
        }
    }

    public void updateDriveTrain() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double speed = Math.hypot(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
        double angle = Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - (Math.PI/4);
        angle += angles.firstAngle + STARTING_HEADING;
        double turnPower = opMode.gamepad1.right_stick_x;

        if(turnPower == 0){
            if (count < 10) {
                angleTest[count] = angle;
                angleTest[count] = angle;
                count++;
            }
            else if (count >= 10){
                sum = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0] + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8] + angleTest[9])/10;
                if(sum > angle){
                    correct = sum - angle;
                    angle = angle + correct;
                }
                else if(angle > sum){
                    correct = angle - sum;
                    angle = angle - correct;
                }
                count = 0;

            }
        }

        motorDriveLF.setPower(((speed * -(Math.cos(angle)) + turnPower)));
        motorDriveLB.setPower(((speed * -(Math.sin(angle)) + turnPower)));
        motorDriveRF.setPower(((speed * (Math.sin(angle))) + turnPower));
        motorDriveRB.setPower(((speed * (Math.cos(angle))) + turnPower));

    }

    public void controlCollection() {//In case of emergency, driver can take over collection controls
        //collect if left trigger pressed
        //needs to be fancified but I don't know how :D

        if (driveCollect == false && opMode.gamepad1.dpad_left){
            driveCollect = true;
        }
        if (driveCollect == true && opMode.gamepad1.dpad_right){
            driveCollect = false;
        }

        if (driveCollect == true){
            if (opMode.gamepad1.left_trigger > .3)
            {
                motorCollectionL.setPower(-.3 * opMode.gamepad1.left_trigger);
                motorCollectionR.setPower(.3 * opMode.gamepad1.left_trigger);
            }
            //eject if right trigger is pressed
            //needs to be fancified but I don't know how :D
            else if (opMode.gamepad1.right_trigger > .3) {
                motorCollectionL.setPower(.5 * opMode.gamepad1.right_trigger);
                motorCollectionR.setPower(-.5 * opMode.gamepad1.right_trigger);
            }

            else {
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
            }
        }
        else{
            if (opMode.gamepad2.left_trigger > .3)
            {
                motorCollectionL.setPower(-.3 * opMode.gamepad2.left_trigger);
                motorCollectionR.setPower(.3 * opMode.gamepad2.left_trigger);
            }
            //eject if right trigger is pressed
            //needs to be fancified but I don't know how :D
            else if (opMode.gamepad2.right_trigger > .3) {
                motorCollectionL.setPower(.5 * opMode.gamepad2.right_trigger);
                motorCollectionR.setPower(-.5 * opMode.gamepad2.right_trigger);
            }

            else {
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
            }
        }
    }


    public void controlFoundationServos (){ //Control the foundation servos
        if (opMode.gamepad1.left_bumper){
            servoFoundationL.setPosition(1);
            servoFoundationR.setPosition(0);
        }
        if (opMode.gamepad1.right_bumper){
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
        }
    }

    public void parkServo(){ //Extend our parking mechanism
        if (opMode.gamepad2.dpad_down) {
            servoPark.setPower(1);
        }
        else if (opMode.gamepad2.dpad_up){
            servoPark.setPower(-1);
        }
        else {
            servoPark.setPower(0);
        }
    }

    public void controlLift(double rightStick, double leftStick) { //Control lift motors and linkage servos
        if (opMode.gamepad2.right_bumper){
            motorLiftL.setPower(.05);
            motorLiftR.setPower(.05);
        }
        else {
            if(Math.signum(-rightStick) == -1) { // Down
                motorLiftL.setPower(-rightStick/2);
                motorLiftR.setPower(-rightStick/2);
            } else {                             // Up
                motorLiftL.setPower(-rightStick);
                motorLiftR.setPower(-rightStick);
            }
        }

        servoFourbarArmL.setPower(-leftStick);
        servoFourbarArmR.setPower(leftStick);
    }

    public void decideAutoTransfer(){
        if (opMode.gamepad2.x && autoTransfer == false){
            autoTransfer = true;
        }
        if (opMode.gamepad2.b && autoTransfer == true){
            autoTransfer = false;
        }
    }

    public void grabStone(){ //Automatically grab stones once they're in the hopper

        if (sensorColor.getDistance(DistanceUnit.CM) >= 2){
            opMode.telemetry.addData("State: ", "Empty");
            hopperState = 0;
        }
        if (sensorColor.getDistance(DistanceUnit.CM) < 2){
            opMode.telemetry.addData("State: ", "Stone");
            hopperState = 1;
        }
        opMode.telemetry.update();

        switch(transfer.ordinal())  {
            case 0:
                if (hopperState == 1 && transfer == TransferState.IDLE && autoTransfer == true){ //If there is a stone in the hopper,
                    transfer = TransferState.LOWERING_LINKAGE;                                   //it hasn't already grabbed something,
                    transferTimer.reset();                                                       // and we want to collect automatically,
                }                                                                                //enter the state machine to automatically grab it
                break;
            case 1:
                servoFourbarArmL.setPower(-.3);
                servoFourbarArmR.setPower(.3);
                if (transferTimer.seconds() >= .6){ //Lower the linkage onto the stone
                    transfer = TransferState.GRABBING;
                    transferTimer.reset();
                }
                break;
            case 2:
                servoGrabber.setPosition(0); //Grab the stone
                transfer = TransferState.GRABBED;
                break;
            case 3:
                if (opMode.gamepad2.y){ //Once We open the grabber, reset the system so we can repeat
                    transfer = TransferState.IDLE;
                }
        }
    }

    public void controlServoClaw(){
        if(opMode.gamepad2.y || opMode.gamepad1.y) {
            servoGrabber.setPosition(1);
            transfer = TransferState.IDLE; //When we open the grabber, we don't have a stone and we can auto grab
        }
        if (opMode.gamepad2.a || opMode.gamepad1.a) {
            servoGrabber.setPosition(0);
            transfer = TransferState.GRABBED; //When we have already grabbed a stone, we can't auto grab
        }
    }

    public void controlCap(){
        if (opMode.gamepad1.dpad_up){
            servoCapstone.setPower(1);
        }
        else if (opMode.gamepad1.dpad_down){
            servoCapstone.setPower(-1);
        }
        else {
            servoCapstone.setPower(0);
        }
    }

    public void switchCollection(){
        if (driveCollect == false && opMode.gamepad1.dpad_up){
            driveCollect = true;
        }
        if (driveCollect == true && opMode.gamepad1.dpad_down){
            driveCollect = false;
        }
    }

    public void postTelemetry() {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        opMode.telemetry.addLine();
        opMode.telemetry.addData("Running for...", timerOpMode.seconds());
        opMode.telemetry.addData("autoTransfer: ", "" + autoTransfer);
        opMode.telemetry.addData("Lift State", transfer);
        opMode.telemetry.addData("Who owns collection?", driveCollect ? "Driver" : "Operator");
        opMode.telemetry.addData("Foundation Grabbed?", isFoundationGrabbed());
        opMode.telemetry.update();
    }

}