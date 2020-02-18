package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class SteveBase {
    // OpMode Class
    OpMode opMode;
    // Every single device on the robot
    DcMotor motorDriveLF;            // Front-left drive motor
    DcMotor motorDriveLB;            // Back-left drive motor
    DcMotor motorDriveRF;            // Front-right drive motor
    DcMotor motorDriveRB;            // Back-right drive motor
    DcMotor motorCollectionL;        // Left collection intake motor
    DcMotor motorCollectionR;        // Right collection intake motor
    DcMotor motorLiftL;              // Left lift motor
    DcMotor motorLiftR;              // Right lift motor
    Servo servoFoundationL;          // Left foundation grabber servo
    Servo servoFoundationR;          // Right foundation grabber servo
    Servo servoGrabber;              // Stone's nub grabber servo
    CRServo servoFourbarArmL;        // Left Fourbararm servo on the lift
    CRServo servoFourbarArmR;        // Right Fourbararm servo on the lift
    CRServo servoPark;               // Parking mechanism servo
    DistanceSensor sensorColor;      // Hopper's proximity sensor (Detects stones in the hopper)
    ColorSensor sensorSkystones;     // Color Sensor for finding Skystones
    DigitalChannel foundationTouchR; // Left foundation Touch Sensor
    DigitalChannel foundationTouchL; // Right foundation Touch Sensor
    BNO055IMU imu;                   // IMU Gyro itself
    Orientation angles;              // IMU Gyro's Orienting
    // Stores the robot's heading at the end of autonomous to be used in teleop
    File headingFile = AppUtil.getInstance().getSettingsFile("headingFile");
    // Timers
    ElapsedTime timerOpMode;         // Tells us how long the Opmode is running since it started.
    ElapsedTime timerTravel;         // Used to cancel a robot turn in case we lock up.
    ElapsedTime timerTransferState;  // Keeps track of time in our automatic transfer system
    // Variables used in Path Selection
    String allianceColor;                                     // What's our alliance color?
    String parkingPreference;                                 // Where do we want to park at the end?
    boolean scoreSkyStones = false;                           // Score Skystones?
    boolean pushingFoundation = false;                        // Or grab the foundation?
    String skystonePosition = "";                             // What's the skystone position?
    String[] skystonePositions = {"LEFT", "CENTER", "RIGHT"}; // All possible stone positions
    // Variables used in Odometry
    double leftWheelTickDelta = 0;                            // Change in left encoder rotation
    double rightWheelTickDelta = 0;                           // Change in right encoder rotation
    double strafeWheelTickDelta = 0;                          // Change in strafing encoder rotation
    double leftWheelTickDeltaPrevious = 0;                    // Last change in left encoder rotation
    double rightWheelTickDeltaPrevious = 0;                   // Last change in right encoder rotation
    double strafeWheelTickDeltaPrevious = 0;                  // Last change in strafing encoder rotation
    double xPosition = 0;                                     // Robot's x-position from starting point
    double yPosition = 0;                                     // Robot's y-position from starting point
    double initialHeading = 90;                               // Robot's starting heading
    double robotHeading = 0;                                  // Robot's current heading
    double robotHeadingDelta = 0;                             // Change in robot heading
    double robotHeadingPrevious = 0;                          // Last change in robot heading
    double robotSpeedInFPS;                                   // Robot speed in Feet per second
    // Variables used in Autonomous
    double attemptsToGrabFoundation = 0;         // How many times have we grabbed the foundation?
    // Variables used in the Teleop drive code
    double angleTest[] = new double[10];         // Last 10 robot headings
    int count = 0;                               // Iterates through the last 10 robot headings
    double sum;                                  // Sum of the last 10 robot headings
    double correct;                              // Angle correction to deal with robot drift
    // Variables used in the Teleop operator code
    boolean driveCollect = false;                // Does the driver have collection controls?
    boolean autoTransferEnabled = true;          // Is the automatic transfer system enabled?
    int hopperState = 0;                         // Is there a stone in the hopper? (1 for full)
    int liftPositionLock;                        // Height of the lift locking position
    boolean liftLockInPlace;                     // Is the lift lock in place?
    // CONSTANTS
    double ENCODER_CPR = 360;                    // Encoder CPR
    double WHEEL_CURCUMFERENCE = 2.28;           // Wheel circumference of the encoder wheels
    double COUNTS_PER_INCH = ENCODER_CPR / WHEEL_CURCUMFERENCE; // Calculated Counts per Inch on the encoders
    double STARTING_HEADING = 0;                 // Robot Starting Heading
    double STRAFE_WHEEL_OFFSET_CONSTANT = 354;   // Deals with the strafing encoder moving from robot turns
    double TARGET_POSITION_ACCURACY_IN_INCHES = 0.5; // Accuracy constant for robot travel
    double WAYPOINT_POSITION_ACCURACY_IN_INCHES = 2; // Accuracy constant for robot travel
    double TARGET_HEADING_ACCURACY_IN_DEGREES = 2;   // Accuracy constant for robot turning
    double SKYSTONE_DETECTION_THRESHHOLD = 60;       // Threshold between the skystone's and stone's light reflections
    // State Machine
    private enum TransferState {                     // Automatic Transfer System (ATS) steps
        IDLE, LOWERING_LINKAGE, GRABBING, GRABBED
    }
    TransferState transfer = TransferState.IDLE;     // What step is the ATS on?

    /** Sets up the Base Class within the OpMode
     * @param theOpMode the OpMode Class (usually just "this")
     * */
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
        timerTransferState = new ElapsedTime();

        opMode.telemetry.addLine("Initialization Succeeded!");
        opMode.telemetry.update();

    }

    /** Just gonna reset the encoders real quick...*/
    public void resetEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** ... And now we set them to run without the encoders!
     * (NOTE: they will still return position, but there's no fancy PID control on the encoders.)*/
    public void runWithoutEncoders(){
        motorDriveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Set power to ALL drivetrain motors. Uses setDrivePowerSides() for simplicity.
     * @param motorPower the power given to each motor
     * */
    public void setDrivePower(double motorPower) {
        setDrivePowerSides(motorPower, motorPower);
    }

    /** Set power to the right and left drivetrain sides individually.
     * Uses setDrivePowerMotors() for simplicity.
     * @param motorPowerL the power given to the left side of the drivetrain
     * @param motorPowerR the power given to the right side of the drivetrain
     * */
    public void setDrivePowerSides(double motorPowerL, double motorPowerR) {
        setDrivePowerMotors(motorPowerL, motorPowerL, motorPowerR, motorPowerR);
    }
    /** Set unique powers to each of the drivetrain motors.
     * @param motorPowerLF the power given to the front-left drive motor
     * @param motorPowerLB the power given to the back-left drive motor
     * @param motorPowerRF the power given to the front-right drive motor
     * @param motorPowerRB the power given to the back-right drive motor
     * */
    public void setDrivePowerMotors(double motorPowerLF, double motorPowerLB, double motorPowerRF, double motorPowerRB) {
        motorDriveLF.setPower(motorPowerLF);
        motorDriveLB.setPower(motorPowerLB);
        motorDriveRF.setPower(motorPowerRF);
        motorDriveRB.setPower(motorPowerRB);
    }

    /* =======================AUTONOMOUS EXCLUSIVE METHODS========================= */
    /** Is the foundation pressing either of the touch sensors beside the grabbers?
     * @return true for either being pressed or false for not pressed.
     * */
    public boolean isFoundationDetected() {
        // Since getState() returns false for pressed and true for not pressed, we had to apply some logic inverters to return results that make sense.
        return (!foundationTouchL.getState() || !foundationTouchR.getState());
    }

    /** Simplifies the code without having to use ((LinearOpMode)opMode).sleep(ms) all the time
     * @param ms how many milliseconds we want to pause for.
     * */
    public void sleep(long ms) {
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }

    /** Another simplification function, this one checks if the robot is initializing.
     * @return whether or not we have pressed init on the driver station.
     * */
    boolean isInitialized() {
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }

    /** We use the driver’s gamepad controller to select autonomous options.
     * The drive team can select the alliance color, parking preference, and whether to pull the
     * foundation, deliver the Skystones, or neither. It then displays the results while we wait
     * for the match to begin.*/
    public void selection() {
        // What alliance color are we? (By the way, that loop is a do-while loop. It's like a while
        // loop, but it always runs at least once because of how it's set up)
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

        // Autonomous Strategy?
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

        // Display the results.
        opMode.telemetry.addData("All Set! Just press PLAY when ready!", "D");
        opMode.telemetry.addData("Current Alliance", allianceColor);
        opMode.telemetry.addData("Parking Preference", parkingPreference);
        opMode.telemetry.addData("Score SkyStone in Auto?", scoreSkyStones);
        opMode.telemetry.addData("Move Foundation in Auto?", pushingFoundation);
        opMode.telemetry.update();
    }

    /** Intakes the motors to be used by each of the encoders, and then updates robot position based
     * on how much change has occured in the motor wheels and robot heading.
     * @param encoderLeft usually uses motorDriveLB
     * @param encoderRight usually uses motorDriveRB
     * @param encoderStrafe usually uses motorDriveLF
     * */
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

        opMode.telemetry.addData("Robot Speed in Feet per Second", robotSpeedInFPS);
        opMode.telemetry.update();
    }

    /** Tells the robot to travel to a certain position and heading on the field while constantly
     * updating position and heading.
     * @param xTarget the x-value of the target position relative to starting position in inches.
     * @param yTarget the y-value of the target position relative to starting position in inches.
     * @param lockedHeading what robot heading we want to end up at
     * @param targetAccuracy how close we want to be to the target before we stop.
     * @param powerBoost Optional parameter for cases when we need lots of pushing power
     *                   (*stares at the foundation*)
     * */
    void travelToPosition(double xTarget, double yTarget, double lockedHeading, double targetAccuracy, double... powerBoost) {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        double offsetHeading = lockedHeading - robotHeading;
        double xDelta = -xTarget + xPosition;
        double yDelta = yTarget - yPosition;

        while(((Math.abs(xDelta) > targetAccuracy ||
                Math.abs(yDelta) > targetAccuracy) ||
                (Math.abs(offsetHeading) > TARGET_HEADING_ACCURACY_IN_DEGREES) &&
                        ((LinearOpMode)opMode).opModeIsActive())) {
            updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
            xDelta = -xTarget + xPosition;
            yDelta = yTarget - yPosition;
            offsetHeading = lockedHeading - robotHeading;
            offsetHeading += offsetHeading > 120 ? -360 :
                    offsetHeading < -240 ? 360 : 0;

            double desiredSpeedLF = 0;
            double desiredSpeedLB = 0;
            double desiredSpeedRF = 0;
            double desiredSpeedRB = 0;

            if(Math.abs(xDelta) > targetAccuracy ||
                    Math.abs(yDelta) > targetAccuracy) {
                // Determines wheel power for each motor based on our base motor power and target X, Y, and Theta values
                double angleDirection = Math.atan2(yDelta, xDelta) + Math.toRadians(robotHeading) - Math.toRadians(initialHeading);
                double powerMod = powerBoost.length > 0 ? powerBoost[0] : 0; // For when we need heavy duty pushing power
                double powerCurve = Math.sqrt(Math.abs(Math.hypot(xDelta, yDelta))/80);
                double drivingPower = Range.clip(powerCurve + 0.1 + powerMod, 0, 0.55);
                double wheelTrajectory = angleDirection - (Math.PI/4);

                desiredSpeedLF -= (drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2);
                desiredSpeedLB -= (drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2);
                desiredSpeedRF += (drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2);
                desiredSpeedRB += (drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2);
            }
            // Correct for robot drifting
            if(Math.abs(offsetHeading) > TARGET_HEADING_ACCURACY_IN_DEGREES) {
                double turnMod = Range.clip(Math.toRadians(offsetHeading * 1.5), -0.5, 0.5);
                desiredSpeedLF = Range.clip(desiredSpeedLF - turnMod, -1, 1);
                desiredSpeedLB = Range.clip(desiredSpeedLB - turnMod, -1, 1);
                desiredSpeedRF = Range.clip(desiredSpeedRF - turnMod, -1, 1);
                desiredSpeedRB = Range.clip(desiredSpeedRB - turnMod, -1, 1);
            }

            setDrivePowerMotors(desiredSpeedLF, desiredSpeedLB, desiredSpeedRF, desiredSpeedRB);
        }
        setDrivePower(0);
        imuTurn(offsetHeading);
    }

    /** The IMU Turn we used last year. Not as complicated as the encoder drive, yet more
     * complicated at the same time. Enjoy.
     * @param degreesToTurn how many degrees we want to turn. And no, I'm not a radian guy.
     * */
    private void imuTurn(double degreesToTurn) {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        double currentHeading = angles.firstAngle + 180;
        double targetHeading = degreesToTurn + currentHeading;
        targetHeading += targetHeading > 360 ? -360 :
                targetHeading < 0 ? 360 : 0;

        timerTravel.reset();
        while (Math.abs(degreesToTurn) > 5 && ((LinearOpMode)opMode).opModeIsActive() && timerTravel.seconds() <= 2) {
            updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
            currentHeading = angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            double TURN_POWER = Range.clip(Math.signum(degreesToTurn) * (0.2 + (Math.abs(degreesToTurn) / 270)), -0.35, 0.35);
            setDrivePowerSides(-TURN_POWER, -TURN_POWER);
        }
        setDrivePower(0);
        sleep(200);
    }

    /** Called in the OpMode to grab the foundation, readying the robot to score it.
     * Behaves differently based on autonomous path.
     * */
    public void grabFoundation() {
        if(allianceColor.equals("RED")) {
            travelToPosition(13, 26, 0, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
            travelToPosition(13, 31, 0, TARGET_POSITION_ACCURACY_IN_INCHES);
            while(((LinearOpMode)opMode).opModeIsActive() && attemptsToGrabFoundation < 6 && timerOpMode.seconds() < 29) {
                servoFoundationL.setPosition(0);
                servoFoundationR.setPosition(1);
                attemptsToGrabFoundation++;
                sleep(1000);
                travelToPosition(13, 31 + (attemptsToGrabFoundation * 2), 0, TARGET_POSITION_ACCURACY_IN_INCHES);
                if(isFoundationDetected()) {
                    break;
                }
                else if(((LinearOpMode)opMode).opModeIsActive()){
                    servoFoundationL.setPosition(1);
                    servoFoundationR.setPosition(0);
                    sleep(600);
                    attemptsToGrabFoundation++;
                    travelToPosition(13, 31 + (attemptsToGrabFoundation * 2), 0, TARGET_POSITION_ACCURACY_IN_INCHES);
                }
            }
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
            sleep(800);
            // FOUNDATION GRASPED
            travelToPosition(15, yPosition, 0, TARGET_POSITION_ACCURACY_IN_INCHES);
            travelToPosition(15, 31, 0, TARGET_POSITION_ACCURACY_IN_INCHES);
        } else if(allianceColor.equals("BLUE")) {
            travelToPosition(-13, 26, 0, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
            travelToPosition(-13, 31, 0, TARGET_POSITION_ACCURACY_IN_INCHES);
            while(((LinearOpMode)opMode).opModeIsActive() && attemptsToGrabFoundation < 6 && timerOpMode.seconds() < 29) {
                servoFoundationL.setPosition(0);
                servoFoundationR.setPosition(1);
                sleep(1000);
                attemptsToGrabFoundation++;
                travelToPosition(-13, 31 + (attemptsToGrabFoundation * 2), 0, TARGET_POSITION_ACCURACY_IN_INCHES);
                if(isFoundationDetected()) {
                    break;
                }
                else if(((LinearOpMode)opMode).opModeIsActive()){
                    servoFoundationL.setPosition(1);
                    servoFoundationR.setPosition(0);
                    sleep(600);
                    attemptsToGrabFoundation++;
                    travelToPosition(-13, 31 + (attemptsToGrabFoundation * 2), 0, TARGET_POSITION_ACCURACY_IN_INCHES);
                }
            }
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
            sleep(800);
            // FOUNDATION GRASPED
            travelToPosition(-15, yPosition, 0, TARGET_POSITION_ACCURACY_IN_INCHES);
            travelToPosition(-15, 31, 0, TARGET_POSITION_ACCURACY_IN_INCHES);
        }
    }

    /** Called in the OpMode to pull the foundation in a curving motion, and pushing it.
     * Behaves differently based on autonomous path.
     * */
    public void turnAndScoreFoundation(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(allianceColor.equals("RED")){
            while (angles.firstAngle > -85 && ((LinearOpMode)opMode).opModeIsActive() && timerOpMode.seconds() < 29){
                updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
                setDrivePowerSides(1, -0.3);
            }
            travelToPosition(3, 5, 0, TARGET_POSITION_ACCURACY_IN_INCHES, 0.1);
        } else if(allianceColor.equals("BLUE")){
            while (angles.firstAngle < 85 && ((LinearOpMode)opMode).opModeIsActive() && timerOpMode.seconds() < 29){
                updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
                setDrivePowerSides(0.3, -1);
            }
            travelToPosition(-4, 5, 0, TARGET_POSITION_ACCURACY_IN_INCHES, 0.1);
        }
        servoFoundationL.setPosition(1);
        servoFoundationR.setPosition(0);
        sleep(600);
        // FOUNDATION MOVED TO BUILDING SITE
    }

    /** Called in the OpMode to drive up to the stones in the loading zone and identify which one is
     * the SkyStone.
     * Behaves differently based on autonomous path.
     * */
    void lookForSkystones() {
        if(allianceColor.equals("RED")) {
            for(int position = 0; position < 2; position++) { // Going from left to right
                travelToPosition(-28, -3 + (8 * position), 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                opMode.telemetry.addData("Alpha", sensorSkystones.alpha());
                opMode.telemetry.update();
                if(sensorSkystones.alpha() < SKYSTONE_DETECTION_THRESHHOLD) {
                    skystonePosition = skystonePositions[position];
                    return;
                }
            }
            // If Skystone isn't found, we can assume it's the right stone because we haven't checked it yet.
            skystonePosition = "RIGHT";
        } else if(allianceColor.equals("BLUE")) {
            for(int position = 0; position < 2; position++) { // Going from right to left
                travelToPosition(-28, 8 * position, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                if(sensorSkystones.alpha() < SKYSTONE_DETECTION_THRESHHOLD) {
                    skystonePosition = skystonePositions[position];
                    return;
                }
            }
            // If Skystone isn't found, just grab the right stone because it's the easiest to intake.
            skystonePosition = "RIGHT";
        }
    }

    /** Called in the OpMode to intake each of the two skystones and spit them out into the
     * building zone individually.
     * Behaves differently based on autonomous path.
     * */
    public void collectAndScoreSkystones(){
        lookForSkystones();
        if(allianceColor.equals("RED")) {
            if(skystonePosition.equals("LEFT")) {
                double stonePosX = -39;
                double stonePosY = 11;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;

                travelToPosition(xPosition, stonePosY, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 4, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // FIRST SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, 53 ,-90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // FIRST SKYSTONE SCORED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 15, stonePosY - 24, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY - 24, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 28, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // SECOND SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, 53, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // SECOND SKYSTONE SCORED
            } else if(skystonePosition.equals("CENTER")) {
                double stonePosX = -39;
                double stonePosY = 19;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;

                travelToPosition(xPosition, stonePosY, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 4, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // FIRST SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, 53, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // FIRST SKYSTONE SCORED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 15, stonePosY - 24, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY - 24, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 28, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // SECOND SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, 53 ,-90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // SECOND SKYSTONE SCORED
            } else if(skystonePosition.equals("RIGHT")) {
                double stonePosX = -38.5;
                double stonePosY = 27;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;

                travelToPosition(xPosition + 1, stonePosY-4, 45, TARGET_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 5, stonePosY-4, 45, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX - 1, stonePosY - 10, 55, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // FIRST SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, 53, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // FIRST SKYSTONE SCORED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 15, stonePosY - 24, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY - 24, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY - 29, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // SECOND SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, 16, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, 53, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // SECOND SKYSTONE SCORED
            }
            // BLUE SIDE
        } else if(allianceColor.equals("BLUE")) {
            if (skystonePosition.equals("LEFT")) {
                double stonePosX = -43;
                double stonePosY = -13;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;
                travelToPosition(stonePosX + 4, yPosition, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 4, stonePosY + 6, -45, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 2, yPosition, -45, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY + 8, -45, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // FIRST SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, -40, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // FIRST SKYSTONE SCORED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 15, stonePosY + 24, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY + 24, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY + 28, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // SECOND SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, -40, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // SECOND SKYSTONE SCORED
            } else if (skystonePosition.equals("CENTER")) {
                double stonePosX = -44;
                double stonePosY = -5;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;
                travelToPosition(xPosition + 3, yPosition, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(xPosition + 3, stonePosY, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY + 4, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // FIRST SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, -40, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // FIRST SKYSTONE SCORED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 15, stonePosY + 24, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY + 24, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY + 29, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // SECOND SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, -40, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // SECOND SKYSTONE SCORED
            } else if (skystonePosition.equals("RIGHT")) {
                double stonePosX = -43;
                double stonePosY = 4;
                double skybridgePassingX = parkingPreference.equals("INSIDE") ? -26.5 : -5;

                travelToPosition(stonePosX + 15, stonePosY, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX, stonePosY + 4, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // FIRST SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, -40, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // FIRST SKYSTONE SCORED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX + 15, stonePosY + 24, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(stonePosX, stonePosY + 24, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(1);
                motorCollectionR.setPower(-1);
                travelToPosition(stonePosX + 4, stonePosY + 29, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                sleep(200);
                motorCollectionL.setPower(0);
                motorCollectionR.setPower(0);
                // SECOND SKYSTONE COLLECTED
                travelToPosition(skybridgePassingX, -10, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                travelToPosition(skybridgePassingX, -40, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                motorCollectionL.setPower(-0.35);
                motorCollectionR.setPower(0.35);
                // SECOND SKYSTONE SCORED
            }
        }
    }

    /** Called in the OpMode to park under the Skybridge at the end of autonomous.
     * Behaves differently based on autonomous path.
     * */
    public void driveToSkybridge() {
        if(timerOpMode.seconds() < 29) {
            if(scoreSkyStones) {
                if (allianceColor.equals("RED")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(-26.5, 45, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(-5, 45, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    }
                } else if (allianceColor.equals("BLUE")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(-26.5, -30, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(-5, -30, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    }
                }
            } else {
                if (allianceColor.equals("RED")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(-20, 43, -90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                        travelToPosition(-40, 43, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(-40, 6, -90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    }
                } else if (allianceColor.equals("BLUE")) {
                    if (parkingPreference.equals("INSIDE")) {
                        travelToPosition(18, 43, 90, WAYPOINT_POSITION_ACCURACY_IN_INCHES);
                        travelToPosition(35, 43, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    } else if (parkingPreference.equals("OUTSIDE")) {
                        travelToPosition(35, 5, 90, TARGET_POSITION_ACCURACY_IN_INCHES);
                    }
                }
            }
        }
    }

    /** Wait for the end of autonomous to store our heading.*/
    public void waitForEnd(){
        while (timerOpMode.seconds() <29){ // Yes. It is, in fact, an empty loop.
        }
    }

    /** At the end of the autonomous period, the robot stores the heading of the robot in a file
     * setting. The robot will retrieve this heading at the start of autonomous to be used in the
     * field-centric driver controls. */
    public void storeHeading(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        ReadWriteFile.writeFile(headingFile, String.valueOf(-angles.firstAngle));
        opMode.telemetry.addData("headingFile", "" + ReadWriteFile.readFile(headingFile)); //Store robot heading to phone
        opMode.telemetry.update();
    }

    /* =======================TELEOP METHODS========================= */

    /** At the start of the program, the robot retrieves the robot’s heading stored in a file
     * setting on the phone during Autonomous. This is used in our Field-Centric Drive Control
     * of the robot.*/
    public void retrieveHeading() {
        try {
            STARTING_HEADING = Double.parseDouble(ReadWriteFile.readFile(headingFile));
            opMode.telemetry.addData("Heading retrieved", STARTING_HEADING);
            opMode.telemetry.update();
        } catch(Exception exc) {
            opMode.telemetry.addData("ERROR", "Couldn't read headingFile; will set startingHeading to 0.");
            opMode.telemetry.update();
            STARTING_HEADING = 0;
        }
    }

    /** In the event that the teleop retrieves a faulty robot heading without running autonomous
     * first, the driver can simply realign the robot with the field and set that heading as the
     * starting heading, satisfying the Field-Centric Drive Control of the robot.*/
    public void resetHeading(){ // Resets the imu heading by adding/subtracting from itself
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

    /** Updates drivetrain motor powers based on the driver's right and left joysticks.
     * Uses Field-centric Driver control for easier driving.*/
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

    /** Updates our compliant-wheel intake system based on the operator's left and right triggers.
     * The driver can steal collection controls in the event that the operator gamepad disconnects
     * or if we need to test collection with one gamepad controller.*/
    public void controlCollection() {
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

    /** Updates our foundation grabber positions with the operator's left and right bumpers.*/
    public void controlFoundationServos (){
        if (opMode.gamepad1.left_bumper){
            servoFoundationL.setPosition(1);
            servoFoundationR.setPosition(0);
        }
        if (opMode.gamepad1.right_bumper){
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
        }
    }

    /** Updates our parking mechanism with the operator's dpad.*/
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

    /** Updates our lift and its fourbar arm based on the operator's joysticks.*/
    public void controlLift(double rightStick, double leftStick) { //Control lift motors and linkage servos
        if(rightStick == 0){
            if(!liftLockInPlace){
                liftPositionLock = motorLiftL.getCurrentPosition();
                liftLockInPlace = true;
            }
            int liftPositionError = liftPositionLock - motorLiftL.getCurrentPosition();
            motorLiftL.setPower(liftPositionError);
            motorLiftR.setPower(liftPositionError);
        }
        else {
            liftLockInPlace = false;
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

    /** Updates whether or not the operator wants to do automatic stone grabbing.*/
    public void decideAutoTransfer(){
        if (opMode.gamepad2.x && autoTransferEnabled == false){
            autoTransferEnabled = true;
        }
        if (opMode.gamepad2.b && autoTransferEnabled == true){
            autoTransferEnabled = false;
        }
    }

    /** Automatic stone grabbing using a "state machine" and  the proximity sensor built into the
     * hopper*/
    public void grabStone(){
        // Updates the proximity sensor's readings.
        if (sensorColor.getDistance(DistanceUnit.CM) >= 2){
            hopperState = 0;
        }
        if (sensorColor.getDistance(DistanceUnit.CM) < 2){
            hopperState = 1;
        }
        // State machines allow us to run a sequence program within an updating loop.
        switch(transfer.ordinal())  {
            // If there is a stone in the hopper,
            // it hasn't already grabbed something,
            // and we want to collect automatically,
            // enter the state machine to automatically grab it
            case 0:
                if (hopperState == 1 && transfer == TransferState.IDLE && autoTransferEnabled){
                    transfer = TransferState.LOWERING_LINKAGE;
                    timerTransferState.reset();
                }
                break;
            // Lower the fourbar arm on the lift.
            case 1:
                servoFourbarArmL.setPower(-.3);
                servoFourbarArmR.setPower(.3);
                if (timerTransferState.seconds() >= .6){ //Lower the linkage onto the stone
                    transfer = TransferState.GRABBING;
                    timerTransferState.reset();
                }
                break;
            // Grab the stone
            case 2:
                servoGrabber.setPosition(0); //Grab the stone
                transfer = TransferState.GRABBED;
                break;
            // Reset the system once we release the grabber.
            case 3:
                if (opMode.gamepad2.y){ //Once We open the grabber, reset the system so we can repeat
                    transfer = TransferState.IDLE;
                }
        }
    }

    /** Updates our lift's stone grabber position. Can be used by both the driver and operator.*/
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

    /** Changes who gets to control the compliant wheel intake system.*/
    public void switchCollection(){
        if (driveCollect == false && opMode.gamepad1.dpad_up){
            driveCollect = true;
        }
        if (driveCollect == true && opMode.gamepad1.dpad_down){
            driveCollect = false;
        }
    }

    /** All telemetry readings are posted down here.*/
    public void postTelemetry() {
        updateOdometry(motorDriveLB, motorDriveRB, motorDriveLF);
        opMode.telemetry.addLine();
        opMode.telemetry.addData("Running for...", timerOpMode.seconds());
        opMode.telemetry.addData("autoTransferEnabled: ", "" + autoTransferEnabled);
        opMode.telemetry.addData("Lift State", transfer);
        opMode.telemetry.addData("Who owns collection?", driveCollect ? "Driver" : "Operator");
        opMode.telemetry.addData("Foundation Grabbed?", isFoundationDetected());
        opMode.telemetry.update();
    }
}