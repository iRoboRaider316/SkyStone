package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class SteveBase {

    OpMode opMode;
    DcMotor motorDriveLF;
    DcMotor motorDriveLB;
    DcMotor motorDriveRF;
    DcMotor motorDriveRB;

    BNO055IMU imu;                  // IMU Gyro itself
    Orientation angles;             // IMU Gyro's Orienting

    double angleTest[] = new double[10];
    int count = 0;
    double sum;
    double correct;

    String allianceColor;
    boolean scoreSkyStones;
    String parkingPreference;
    boolean pushingFoundation;

    double ENCODER_CPR = 537.6;
    double WHEEL_CURCUMFERENCE = 4 * Math.PI;
    double COUNTS_PER_INCH = ENCODER_CPR / WHEEL_CURCUMFERENCE;
    int ENCODER_DRIVE_ALLOWABLE_ERROR = 7;
    double STARTING_HEADING = 0;

    public SteveBase(OpMode theOpMode) {
        opMode = theOpMode;

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

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);
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

    /* =======================AUTONOMOUS EXCLUSIVE METHODS========================= */
    public void selection() {
        // What alliance color are we? (By the way, this is a do-while loop. It always runs at least once because of how it's set up)
        opMode.telemetry.addData("For blue alliance, press", "X");
        opMode.telemetry.addData("For red alliance, press", "B");
        opMode.telemetry.update();

        do {
            ((LinearOpMode)opMode).sleep(50);
            if(opMode.gamepad1.x) allianceColor = "BLUE";
            if(opMode.gamepad1.b) allianceColor = "RED";
        } while(!opMode.gamepad1.x && !opMode.gamepad1.b);

        // Parking Preference?
        opMode.telemetry.addData("To park inside, press", "Y");
        opMode.telemetry.addData("To park outside, press", "A");
        opMode.telemetry.update();

        do {
            ((LinearOpMode)opMode).sleep(50);
            if(opMode.gamepad1.y) parkingPreference = "INSIDE";
            if(opMode.gamepad1.a) parkingPreference = "OUTSIDE";
        } while(!opMode.gamepad1.y && !opMode.gamepad1.a);

        // Score SkyStones?
        opMode.telemetry.addData("To score the SkyStones, press:", "X");
        opMode.telemetry.addData("Otherwise, press", "B");
        opMode.telemetry.update();

        do {
            ((LinearOpMode)opMode).sleep(50);
            if(opMode.gamepad1.x) scoreSkyStones = true;
            if(opMode.gamepad1.b) scoreSkyStones = false;
        } while(!opMode.gamepad1.x && !opMode.gamepad1.b);

        // Do Foundation Grabbing?
        opMode.telemetry.addData("To move the foundation in auto, press", "Y");
        opMode.telemetry.addData("Otherwise, press", "A");
        opMode.telemetry.update();

        do {
            ((LinearOpMode)opMode).sleep(50);
            if(opMode.gamepad1.y) pushingFoundation = true;
            if(opMode.gamepad1.a) pushingFoundation = false;
        } while(!opMode.gamepad1.y && !opMode.gamepad1.a && !scoreSkyStones);

        opMode.telemetry.addData("All Set! Just press PLAY when ready!", "D");
        opMode.telemetry.addData("Current Alliance", allianceColor);
        opMode.telemetry.addData("Parking Preference", parkingPreference);
        opMode.telemetry.addData("Score SkyStone in Auto?", scoreSkyStones);
        opMode.telemetry.addData("Move Foundation in Auto?", pushingFoundation);
        opMode.telemetry.update();
    }

    void encoderDriveMecanum(double baseMotorPower, double distanceInInchesForward, double distancesInInchesStrafing){ //Distance in inches, x and y as direction values, both between 1- and 1
        // DOING EPIC MATH (Based on Jamari's TeleOp Driving code from last year)
        double distanceInInches = Math.hypot(distancesInInchesStrafing, distanceInInchesForward);
        double angleDirection = Math.atan2(distanceInInchesForward, distancesInInchesStrafing); // May need to catch an ArithmeticException here
        double xTargetDirection = Math.cos(angleDirection);
        double yTargetDirection = Math.sin(angleDirection);

        double drivingPower = baseMotorPower * Math.hypot(xTargetDirection, yTargetDirection);
        double wheelTrajectory = angleDirection - (Math.PI/4);

        // Determining speeds for each drive motor (Multiplied by the sqrt of 2 to make up lost power from the trig functions)
        double desiredSpeedLF = -(drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2);
        double desiredSpeedLB = -(drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2);
        double desiredSpeedRF = (drivingPower * (Math.sin(wheelTrajectory))) * Math.sqrt(2);
        double desiredSpeedRB = (drivingPower * (Math.cos(wheelTrajectory))) * Math.sqrt(2);

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

        opMode.telemetry.addData("position LF", motorDriveLF.getCurrentPosition());
        opMode.telemetry.addData("position LB", motorDriveLB.getCurrentPosition());
        opMode.telemetry.addData("position RF", motorDriveRF.getCurrentPosition());
        opMode.telemetry.addData("position RB", motorDriveRB.getCurrentPosition());
        opMode.telemetry.update();

        /* A loop to keep track of each motor's targets and positions as it runs.
         * To end once the motors hit their targets */
        while(  Math.abs(distanceToTargetLF) > ENCODER_DRIVE_ALLOWABLE_ERROR &&
                Math.abs(distanceToTargetLB) > ENCODER_DRIVE_ALLOWABLE_ERROR &&
                Math.abs(distanceToTargetRF) > ENCODER_DRIVE_ALLOWABLE_ERROR &&
                Math.abs(distanceToTargetRB) > ENCODER_DRIVE_ALLOWABLE_ERROR &&
                ((LinearOpMode)opMode).opModeIsActive()) {

            distanceToTargetLF = newTargetLF - motorDriveLF.getCurrentPosition();
            distanceToTargetLB = newTargetLB - motorDriveLB.getCurrentPosition();
            distanceToTargetRF = newTargetRF - motorDriveRF.getCurrentPosition();
            distanceToTargetRB = newTargetRB - motorDriveRB.getCurrentPosition();

            if(Math.abs(distanceToTargetLF) < ENCODER_DRIVE_ALLOWABLE_ERROR + 400) { motorDriveLF.setPower(desiredSpeedLF / 3); } else {motorDriveLF.setPower(desiredSpeedLF);}
            if(Math.abs(distanceToTargetLB) < ENCODER_DRIVE_ALLOWABLE_ERROR + 400) { motorDriveLB.setPower(desiredSpeedLB / 3); } else {motorDriveLB.setPower(desiredSpeedLB);}
            if(Math.abs(distanceToTargetRF) < ENCODER_DRIVE_ALLOWABLE_ERROR + 400) { motorDriveRF.setPower(desiredSpeedRF / 3); } else {motorDriveRF.setPower(desiredSpeedRF);}
            if(Math.abs(distanceToTargetRB) < ENCODER_DRIVE_ALLOWABLE_ERROR + 400) { motorDriveRB.setPower(desiredSpeedRB / 3); } else {motorDriveRB.setPower(desiredSpeedRB);}

            opMode.telemetry.addData("distanceToTargetLF", distanceToTargetLF);
            opMode.telemetry.addData("distanceToTargetLB", distanceToTargetLB);
            opMode.telemetry.addData("distanceToTargetRF", distanceToTargetRF);
            opMode.telemetry.addData("distanceToTargetRB", distanceToTargetRB);
            opMode.telemetry.update();
        }

        // Shut off motors, because we have arrived, baby!
        setDrivePower(0);
        ((LinearOpMode)opMode).sleep(100);
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
        double targetHeading = degreesToTurn + currentHeading;
        targetHeading += targetHeading > 360 ? -360 :
                         targetHeading < 0 ? 360 : 0;

        while (Math.abs(degreesToTurn) > 2 && ((LinearOpMode)opMode).opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            /*double POWER_CAP = 45;      // Once we hit this power, we should slow down.
            double DEGREES_TO_POWER = Range.clip(Math.pow(degreesToTurn / POWER_CAP, 1/3), -1, 1);
            double TURN_POWER = DEGREES_TO_POWER * .7;*/
            double TURN_POWER = Range.clip(Math.signum(degreesToTurn) * (0.1 + (Math.abs(degreesToTurn) / 270)), -1, 1);
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

        opMode.telemetry.addData("DegreesToTurn", degreesToTurn);
        opMode.telemetry.addData("Target", targetHeading);
        opMode.telemetry.addData("Current", currentHeading);
        opMode.telemetry.addData("position LF", motorDriveLF.getCurrentPosition());
        opMode.telemetry.addData("position LB", motorDriveLB.getCurrentPosition());
        opMode.telemetry.addData("position RF", motorDriveRF.getCurrentPosition());
        opMode.telemetry.addData("position RB", motorDriveRB.getCurrentPosition());
        opMode.telemetry.update();
        setDrivePower(0);
        ((LinearOpMode)opMode).sleep(100);
    }

    public void scoreFoundation() {
        if(allianceColor.equals("RED")) {
            encoderDriveMecanum(0.5, 26, 0); //drive to foundation
            encoderDriveMecanum(0.5, 1, 0); //drive to foundation
            //servoFoundationL.setPosition(1); //grabbing the foundation
            //servoFoundationR.setPosition(0); //the 1's are placeholders cuz they may not be right but they seemed more right to me than zero but I'm prolly way off XD
            ((LinearOpMode)opMode).sleep(800);
            encoderDriveMecanum(-0.5, 0, -4); //drive to foundation
            imuTurn(-90);
            encoderDriveMecanum(-0.6, 0, 20 * Math.sqrt(2));
            encoderDriveMecanum(0.6, 20, 0); //drive to foundation
            //servoFoundationL.setPosition(0);
            //servoFoundationR.setPosition(1);
            ((LinearOpMode)opMode).sleep(300);
        } else if(allianceColor.equals("BLUE")) {
            encoderDriveMecanum(0.4, 26, 0); //drive to foundation
            encoderDriveMecanum(-0.4, 0, -12); //drive to foundation
            encoderDriveMecanum(0.3, 2, 0); //drive to foundation
            //servoFoundationL.setPosition(1); //grabbing the foundation
            //servoFoundationR.setPosition(0); //the 1's are placeholders cuz they may not be right but they seemed more right to me than zero but I'm prolly way off XD
            ((LinearOpMode)opMode).sleep(800);
            encoderDriveMecanum(-0.4, -30, 0);
            encoderDriveMecanum(0.3, 3, 0); //drive to foundation
            //servoFoundationL.setPosition(0);
            //servoFoundationR.setPosition(1);
            ((LinearOpMode)opMode).sleep(300);
        }
    }

    /* =======================TELEOP METHODS========================= */
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

}
