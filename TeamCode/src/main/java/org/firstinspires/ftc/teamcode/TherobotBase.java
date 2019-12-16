package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Just here so we can access all the OpMode-exclusive methods
@Autonomous(name=" ", group = "LinearOpMode")
@Disabled
// Why did I call the robot Therobot? Because it's "The robot". Hopefully we get a better name for League Meet 2...
public class TherobotBase extends LinearOpMode {

    // OPMODE CLASSES
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    // DEVICES
    DcMotor motorDriveLF;
    DcMotor motorDriveLB;
    DcMotor motorDriveRF;
    DcMotor motorDriveRB;
    DcMotor motorCollectionL;
    DcMotor motorCollectionR;
    DcMotor motorLiftT;
    DcMotor motorLiftB;
    Servo servoFoundationL;
    Servo servoFoundationR;
    Servo servoClaw;
    CRServo servoGrabberSwivel;
    CRServo servoCapstone;
    BNO055IMU imu;                  // IMU Gyro itself
    Orientation angles;             // IMU Gyro's Orienting

    ElapsedTime timerOpMode;
    ElapsedTime timerEncoderDrive;

    // VARIABLES
    String allianceColor = "";
    String parkingPreference = "";
    boolean pushingFoundation = false;
    double angleTest[] = new double[10];
    int count = 0;
    double sum;
    double correct;
    double liftPowerShift = 1;

    // CONSTANTS
    double ENCODER_CPR = 537.6;
    double GEAR_REDUCTION = 32 / 24;
    double WHEEL_CURCUMFERENCE = 4 * Math.PI;
    double COUNTS_PER_INCH = ENCODER_CPR * GEAR_REDUCTION / WHEEL_CURCUMFERENCE;
    int ENCODER_DRIVE_ALLOWABLE_ERROR = 20;
    int LIFT_INVERSION_MID_BOUND = 517;
    int LIFT_INVERSION_ARC_EXTENSION = 70;


    public TherobotBase(OpMode opMode) {
        // Initialize the HardwareMap and Telemetry of each OpMode
        // to the ones inherited from each of their base codes.
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        // <INSERT DEVICE INITIALIZATIONS HERE, THE DRIVE MOTORS AND THE IMU ARE DONE FOR YOU>

        motorDriveLF = hardwareMap.dcMotor.get("motorDriveLF");
        motorDriveLB = hardwareMap.dcMotor.get("motorDriveLB");
        motorDriveRF = hardwareMap.dcMotor.get("motorDriveRF");
        motorDriveRB = hardwareMap.dcMotor.get("motorDriveRB");

        motorDriveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        runWithEncoders();

        motorCollectionL = hardwareMap.dcMotor.get("motorCollectionL");
        motorCollectionR = hardwareMap.dcMotor.get("motorCollectionR");

        motorLiftT=hardwareMap.dcMotor.get("motorLiftT");
        motorLiftB=hardwareMap.dcMotor.get("motorLiftB");
        motorLiftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoFoundationL = hardwareMap.servo.get("servoFoundationL");
        servoFoundationR = hardwareMap.servo.get("servoFoundationR");
        servoFoundationL.setPosition(0);
        servoFoundationR.setPosition(1);
        servoClaw = hardwareMap.servo.get("servoClaw");
        servoGrabberSwivel = hardwareMap.crservo.get("servoGrabberSwivel");
        servoCapstone = hardwareMap.crservo.get("servoCapstone");

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);

        timerOpMode = new ElapsedTime();
        timerEncoderDrive = new ElapsedTime();
    }

    public void selection() {
        // What alliance color are we? (By the way, this is a do-while loop. It always runs at least once because of how it's set up)
        telemetry.addData("For blue alliance, press", "X");
        telemetry.addData("For red alliance, press", "B");
        telemetry.update();

        do {
            sleep(50);
            if(gamepad1.x) allianceColor = "BLUE";
            if(gamepad1.b) allianceColor = "RED";
        } while(!gamepad1.x && !gamepad1.b);

        // Parking Preference?
        telemetry.addData("To park inside, press", "Y");
        telemetry.addData("To park outside, press", "A");
        telemetry.update();

        do {
            sleep(50);
            if(gamepad1.y) parkingPreference = "INSIDE";
            if(gamepad1.a) parkingPreference = "OUTSIDE";
        } while(!gamepad1.y && !gamepad1.a);

        // Do Stone Scoring?
        telemetry.addData("To move the foundation in auto, press", "X");
        telemetry.addData("Otherwise, press", "B");
        telemetry.update();

        do {
            sleep(50);
            if(gamepad1.x) pushingFoundation = true;
            if(gamepad1.b) pushingFoundation = false;
        } while(!gamepad1.x && !gamepad1.b);

        telemetry.addData("All Set! Just press PLAY when ready!", "D");
        telemetry.addData("Current Alliance", allianceColor);
        telemetry.addData("Parking Preference", parkingPreference);
        telemetry.addData("Score Stone in Auto?", pushingFoundation);
        telemetry.update();
    }

    // Just here to make the code happy
    @Override
    public void runOpMode() throws InterruptedException {}

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

    public void setCollectionPower(double power) {
        motorCollectionL.setPower(-power);
        motorCollectionR.setPower(power);
    }

    public void setLiftPosition(int position) {
        motorLiftB.setTargetPosition(position);
        motorLiftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(motorLiftB.isBusy()) {
            motorLiftT.setPower(0.5);
            motorLiftB.setPower(0.5);
        }
    }

    /** ====================== AUTONOMOUS METHODS ======================== */

    /** Before you stress out... this isn't here to make you feel bad. But you WILL need it to
     * write the rest of autonomous! As you call the method, just replace all the variable names
     * with whatever you need the autonomous to accomplish!*/
    void encoderDriveMecanum(double baseMotorPower, double distanceInInchesForward, double distancesInInchesStrafing){ //Distance in inches, x and y as direction values, both between 1- and 1
        // DOING EPIC MATH (Based on Jamari's TeleOp Driving code from last year)
        double distanceInInches = Math.hypot(distancesInInchesStrafing, distanceInInchesForward);
        double angleDirection = Math.atan2(distanceInInchesForward, distancesInInchesStrafing); // May need to catch an ArithmeticException here
        double xTargetDirection = Math.cos(angleDirection);
        double yTargetDirection = Math.sin(angleDirection);

        double drivingPower = baseMotorPower * Math.hypot(xTargetDirection, yTargetDirection);
        double wheelTrajectory = angleDirection - (Math.PI/4);

        // Determining speeds for each drive motor (Multiplied by the sqrt of 2 to make up lost power from the trig functions)
        double desiredSpeedLF = (drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2);
        double desiredSpeedLB = (drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2);
        double desiredSpeedRF = -(drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2);
        double desiredSpeedRB = -(drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2);

        // Determining targets for each drive motor
        int newTargetLF = motorDriveLF.getCurrentPosition() + (int)(distanceInInches * desiredSpeedLF * COUNTS_PER_INCH / baseMotorPower);
        int newTargetLB = motorDriveLB.getCurrentPosition() + (int)(distanceInInches * desiredSpeedLB * COUNTS_PER_INCH / baseMotorPower);
        int newTargetRF = motorDriveRF.getCurrentPosition() + (int)(distanceInInches * desiredSpeedRF * COUNTS_PER_INCH / baseMotorPower);
        int newTargetRB = motorDriveRB.getCurrentPosition() + (int)(distanceInInches * desiredSpeedRB * COUNTS_PER_INCH / baseMotorPower);

        // Setting up the distance vairables to keep track of how far we are away from the target
        int distanceToTargetLF = newTargetLF - motorDriveLF.getCurrentPosition();
        int distanceToTargetLB = newTargetLB - motorDriveLB.getCurrentPosition();
        int distanceToTargetRF = newTargetRF - motorDriveRF.getCurrentPosition();
        int distanceToTargetRB = newTargetRB - motorDriveRB.getCurrentPosition();

        // Setting the determined targets to each drive motor
        motorDriveLF.setTargetPosition(newTargetLF);
        motorDriveLB.setTargetPosition(newTargetLB);
        motorDriveRF.setTargetPosition(newTargetRF);
        motorDriveRB.setTargetPosition(newTargetRB);

        timerEncoderDrive.reset();

        /* A loop to keep track of each motor's targets and positions as it runs.
         * To end once the motors hit their targets */
        while(Math.abs(distanceToTargetLF) > ENCODER_DRIVE_ALLOWABLE_ERROR &&
              Math.abs(distanceToTargetLB) > ENCODER_DRIVE_ALLOWABLE_ERROR &&
              Math.abs(distanceToTargetRF) > ENCODER_DRIVE_ALLOWABLE_ERROR &&
              Math.abs(distanceToTargetRB) > ENCODER_DRIVE_ALLOWABLE_ERROR) {

            distanceToTargetLF = newTargetLF - motorDriveLF.getCurrentPosition();
            distanceToTargetLB = newTargetLB - motorDriveLB.getCurrentPosition();
            distanceToTargetRF = newTargetRF - motorDriveRF.getCurrentPosition();
            distanceToTargetRB = newTargetRB - motorDriveRB.getCurrentPosition();

            if(Math.abs(distanceToTargetLF) < ENCODER_DRIVE_ALLOWABLE_ERROR + 100) { motorDriveLF.setPower(desiredSpeedLF / 2); } else {motorDriveLF.setPower(desiredSpeedLF);}
            if(Math.abs(distanceToTargetLB) < ENCODER_DRIVE_ALLOWABLE_ERROR + 100) { motorDriveLB.setPower(desiredSpeedLB / 2); } else {motorDriveLB.setPower(desiredSpeedLB);}
            if(Math.abs(distanceToTargetRF) < ENCODER_DRIVE_ALLOWABLE_ERROR + 100) { motorDriveRF.setPower(desiredSpeedRF / 2); } else {motorDriveRF.setPower(desiredSpeedRF);}
            if(Math.abs(distanceToTargetRB) < ENCODER_DRIVE_ALLOWABLE_ERROR + 100) { motorDriveRB.setPower(desiredSpeedRB / 2); } else {motorDriveRB.setPower(desiredSpeedRB);}

        }

        // Shut off motors, because we have arrived, baby!
        setDrivePower(0);
        sleep(100);
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
                         degreesToTurn < -180 ?  360 : 0;
        return degreesToTurn;
    }

    /** The IMU Turn we used last year. Not as complicated as the encoder drive, yet more
     * complicated at the same time. Enjoy.*/
    public void imuTurn(double degreesToTurn) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle + 180;
        double targetHeading = (degreesToTurn + currentHeading) % 360;

        while (Math.abs(degreesToTurn) > 2) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            /*double POWER_CAP = 45;      // Once we hit this power, we should slow down.
            double DEGREES_TO_POWER = Range.clip(Math.pow(degreesToTurn / POWER_CAP, 1/3), -1, 1);
            double TURN_POWER = DEGREES_TO_POWER * .7;*/
            double TURN_POWER = Range.clip(Math.signum(degreesToTurn) * (0.2 + (Math.abs(degreesToTurn) / 360)), -1, 1);
            setDrivePowerSides(-TURN_POWER, -TURN_POWER);

            telemetry.addData("DegreesToTurn", degreesToTurn);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Current", currentHeading);
            telemetry.update();
        }

        setDrivePower(0);
        sleep(100);
    }

    public void setLiftPower(double powerTop, double powerBottom) {
        motorLiftT.setPower(powerTop);
        motorLiftB.setPower(powerBottom);
    }

    public void moveLiftAuto(int ticks) {
        double POWER = .15;

        int newTarget = motorLiftB.getCurrentPosition() + ticks;
        ticks = newTarget - motorLiftB.getCurrentPosition();
        motorLiftB.setTargetPosition(newTarget);

        while(Math.abs(ticks) > 20) {
            int distanceToMidBound = Math.abs(LIFT_INVERSION_MID_BOUND - motorLiftB.getCurrentPosition());
            ticks = newTarget - motorLiftB.getCurrentPosition();

            if(distanceToMidBound < LIFT_INVERSION_ARC_EXTENSION) {
                setLiftPower(-Math.signum(ticks) * POWER, Math.signum(ticks) * POWER * 1.5);
            } else {
                setLiftPower(Math.signum(ticks) * POWER, Math.signum(ticks) * POWER * 1.5);
            }

        }

        setLiftPower(0, 0);
    }

    public void scoreStone() {
        new Thread(new Runnable() {
            public void run() {
                servoClaw.setPosition(0);
                sleep(300);
                moveLiftAuto(900);
                servoClaw.setPosition(1);
                moveLiftAuto(-1000);
            }
        }).start();
    }

    public void timerCheck() {
        new Thread(new Runnable() {
            public void run() {
                while(timerOpMode.milliseconds() < 29700) { sleep(10);}
                resetEncoders();
                runWithoutEncoders();
                setDrivePower(0);
                stop();
            }
        }).start();
    }

    public void scoreFoundation() {
        if(allianceColor.equals("RED")) {
            encoderDriveMecanum(0.55, 38, 0); //drive to foundation
            encoderDriveMecanum(0.55, 0, 12); //drive to foundation
            encoderDriveMecanum(0.2, 4, 0); //drive to foundation
            servoFoundationL.setPosition(1); //grabbing the foundation
            servoFoundationR.setPosition(0); //the 1's are placeholders cuz they may not be right but they seemed more right to me than zero but I'm prolly way off XD
            sleep(800);
            encoderDriveMecanum(0.55, -52, 0);
            encoderDriveMecanum(0.2, 3, 0); //drive to foundation
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
            sleep(300);
        } else if(allianceColor.equals("BLUE")) {
            encoderDriveMecanum(0.55, 38, 0); //drive to foundation
            encoderDriveMecanum(0.55, 0, -12); //drive to foundation
            encoderDriveMecanum(0.2, 5, 0); //drive to foundation
            servoFoundationL.setPosition(1); //grabbing the foundation
            servoFoundationR.setPosition(0); //the 1's are placeholders cuz they may not be right but they seemed more right to me than zero but I'm prolly way off XD
            sleep(800);
            encoderDriveMecanum(0.55, -52, 0);
            encoderDriveMecanum(0.2, 3, 0); //drive to foundation
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
            sleep(300);
        }
    }

    public void pushTheFoundation() {
        if(allianceColor.equals("RED")){
            encoderDriveMecanum(0.5, 0, -32 * Math.sqrt(2));
            encoderDriveMecanum(0.5, 20, 0);
            encoderDriveMecanum(0.5,0, 15 * Math.sqrt(2));
        }
        else if(allianceColor.equals("BLUE")) {
            encoderDriveMecanum(0.5, 0, 32 * Math.sqrt(2));
            encoderDriveMecanum(0.5, 20, 0);
            encoderDriveMecanum(0.5,0, -15 * Math.sqrt(2));
        }
    }

    public void driveToQuarryAndScoreStonesOnFoundation() {
        if(allianceColor.equals("RED")) {
            encoderDriveMecanum(0.5, 0, -36 * Math.sqrt(2)); // Strafe left of the foundation
            encoderDriveMecanum(0.5, -6, 0);                 // Just going to orient ourselves parallel to the wall
            encoderDriveMecanum(0.5, 30, 0);
            imuTurn(absoluteHeading(-85));
            encoderDriveMecanum(0.5, -44, 0);
            encoderDriveMecanum(0.5, 0, -30);
            // Collect the stone
            setCollectionPower(1);
            setCollectionPower(1);
            moveLiftAuto(200);
            setLiftPower(0.01, 0.01);
            encoderDriveMecanum(0.4, -10, 0);
            encoderDriveMecanum(0.4, 10, 0);
            setLiftPower(-0.1, -0.1);
            servoClaw.setPosition(1);
            sleep(500);
            setCollectionPower(0);
            // Go to score it
            encoderDriveMecanum(0.5, 0, 29);
            encoderDriveMecanum(0.5, 52, 0);
            scoreStone();
            encoderDriveMecanum(0.5, 0, 12);
            sleep(5000);
            setLiftPower(0, 0);
        } else if(allianceColor.equals("BLUE")) {
            encoderDriveMecanum(0.5, 0, 36 * Math.sqrt(2)); // Strafe left of the foundation
            encoderDriveMecanum(0.5, -6, 0);                 // Just going to orient ourselves parallel to the wall
            encoderDriveMecanum(0.5, 30, 0);
            imuTurn(absoluteHeading(85));
            encoderDriveMecanum(0.5, -44, 0);
            encoderDriveMecanum(0.5, 0, 30);
            // Collect the stone
            setCollectionPower(1);
            moveLiftAuto(200);
            setLiftPower(0.01, 0.01);
            encoderDriveMecanum(0.4, -10, 0);
            encoderDriveMecanum(0.4, 10, 0);
            setLiftPower(-0.1, -0.1);
            servoClaw.setPosition(1);
            sleep(500);
            setCollectionPower(0);
            // Go to score it
            encoderDriveMecanum(0.5, 0, -29);
            encoderDriveMecanum(0.5, 52, 0);
            scoreStone();
            sleep(5000);
            setLiftPower(0, 0);
        }

    }

    public void driveToSkybridge() {
        if(pushingFoundation) {
            if(allianceColor.equals("RED")) {
                if(parkingPreference.equals("INSIDE")) {
                    encoderDriveMecanum(0.5, 10, 0);
                    encoderDriveMecanum(0.5, 0, -37 * Math.sqrt(2));
                } else if(parkingPreference.equals("OUTSIDE")) {
                    encoderDriveMecanum(0.5, -20, 0);
                    encoderDriveMecanum(0.5, 0, -37 * Math.sqrt(2));
                }
            } else if(allianceColor.equals("BLUE")) {
                if(parkingPreference.equals("INSIDE")) {
                    encoderDriveMecanum(0.5, 10, 0);
                    encoderDriveMecanum(0.5, 0, 37 * Math.sqrt(2));
                } else if(parkingPreference.equals("OUTSIDE")) {
                    encoderDriveMecanum(0.5, -18, 0);
                    encoderDriveMecanum(0.5, 0, 37 * Math.sqrt(2));
                }
            }
        } else {
            if(allianceColor.equals("RED")) {
                if(parkingPreference.equals("INSIDE")) {
                    encoderDriveMecanum(0.5, 0, -36 * Math.sqrt(2));
                    encoderDriveMecanum(0.5, 30, 0);
                    encoderDriveMecanum(0.5, 0, -15 * Math.sqrt(2));
                } else if(parkingPreference.equals("OUTSIDE")) {
                    encoderDriveMecanum(0.5, 0, -53 * Math.sqrt(2));
                }
            } else if(allianceColor.equals("BLUE")) {
                if(parkingPreference.equals("INSIDE")) {
                    encoderDriveMecanum(0.5, 0, 36 * Math.sqrt(2));
                    encoderDriveMecanum(0.5, 30, 0);
                    encoderDriveMecanum(0.5, 0, 15 * Math.sqrt(2));
                } else if(parkingPreference.equals("OUTSIDE")) {
                    encoderDriveMecanum(0.5, 0, 53 * Math.sqrt(2));
                }
            }
        }
    }


    /** ====================== TELEOP METHODS ======================== */

    public void mecanumDrive(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI/4);
        double turnPower = gamepad1.right_stick_x;

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

        motorDriveLF.setPower(((speed * (Math.cos(angle)) + turnPower)));
        motorDriveRF.setPower(((speed * -(Math.sin(angle))) + turnPower));
        motorDriveLB.setPower(((speed * (Math.sin(angle)) + turnPower)));
        motorDriveRB.setPower(((speed * -(Math.cos(angle))) + turnPower));
    }

    //Collection code
    public void controlCollection() {
        //collect if left trigger pressed
        //needs to be fancified but I don't know how :D
        if (gamepad2.right_trigger > .3) ;
        {
            motorCollectionL.setPower(-.8 * gamepad2.right_trigger);
            motorCollectionR.setPower(.8 * gamepad2.right_trigger);
        }
        //eject if right trigger is pressed
        //needs to be fancified but I don't know how :D
        if (gamepad2.left_trigger > .3) {
            motorCollectionL.setPower(.8 * gamepad2.left_trigger);
            motorCollectionR.setPower(-.8 * gamepad2.left_trigger);
        }
    }

    //nub clamp servo code here
    public void controlServoClaw() {
        if(gamepad2.y || gamepad1.y) {
            servoClaw.setPosition(1);
        }
        if (gamepad2.a || gamepad1.a) {
            servoClaw.setPosition(0);
        }
    }

    //swivel servo code here
    public void controlServoGrabberSwivel (boolean b, boolean x) {
        if (x) {
            servoGrabberSwivel.setPower(1);
        }
        else if (b) {
            servoGrabberSwivel.setPower(-1);
        }
        else {
            servoGrabberSwivel.setPower(0);
        }
    }

    //foundation servos
    public void controlFoundationServos (boolean br, boolean bl){
        if (bl){
            servoFoundationL.setPosition(0);
            servoFoundationR.setPosition(1);
        }
        if (br){
            servoFoundationL.setPosition(1);
            servoFoundationR.setPosition(0);
        }
    }

    public void controlDeliveryArm () {
        if (motorLiftB.getCurrentPosition() > LIFT_INVERSION_MID_BOUND - LIFT_INVERSION_ARC_EXTENSION && motorLiftB.getCurrentPosition()
                < LIFT_INVERSION_MID_BOUND + LIFT_INVERSION_ARC_EXTENSION) {
            liftPowerShift = -0.1;
        } else {
            liftPowerShift = 1;
        }
        if (gamepad2.right_bumper) {
            motorLiftB.setPower(gamepad2.right_stick_y * 0.35);
        } else if (gamepad2.left_bumper) {
            motorLiftB.setPower(.005);
            motorLiftT.setPower(.005);
        } else {
            motorLiftT.setPower(gamepad2.right_stick_y * liftPowerShift * 0.1);
            motorLiftB.setPower(gamepad2.right_stick_y * 0.1);
        }
    }

    public void yeetCapstone(){
        if (gamepad2.dpad_down) {
            servoCapstone.setPower(1);
        }
        else if (gamepad2.dpad_up){
            servoCapstone.setPower(-1);
        }
        else {
            servoCapstone.setPower(0);
        }
    }
}
