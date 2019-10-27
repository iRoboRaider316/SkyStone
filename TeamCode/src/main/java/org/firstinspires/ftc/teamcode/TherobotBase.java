package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Just here so we can access all the OpMode-exclusive methods
@Autonomous(name="This is broken.", group = "LinearOpMode")
@Disabled
// Why did I call the robot Therobot? Because it's "The robot". Hopefully we get a better name for League Meet 2...
public class TherobotBase extends LinearOpMode {

    // OPMODE CLASSES
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    // DEVICES
    // <INSERT DEVICE DECLARATIONS HERE>
    DcMotor motorDriveLF;
    DcMotor motorDriveLB;
    DcMotor motorDriveRF;
    DcMotor motorDriveRB;
    DcMotor motorCollectionL;
    DcMotor motorCollectionR;
    DcMotor motorLiftT;
    DcMotor motorLiftB;
    BNO055IMU imu;                  // IMU Gyro itself
    Orientation angles;             // IMU Gyro's Orienting

    ElapsedTime timerOpMode;

    // VARIABLES
    // <INSERT VARIABLE DECLARATIONS HERE>

    // CONSTANTS
    // <INSERT CONSTANT DECLARATIONS HERE>
    int ENCODER_CPR = 288;
    double GEAR_REDUCTION = 1;
    double WHEEL_CURCUMFERENCE = 8 * Math.PI;
    double COUNTS_PER_INCH = ENCODER_CPR * GEAR_REDUCTION / WHEEL_CURCUMFERENCE;

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

        motorDriveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorCollectionL = hardwareMap.dcMotor.get("motorCollectionL");
        motorCollectionR = hardwareMap.dcMotor.get("motorCollectionR");

        motorLiftT=hardwareMap.dcMotor.get("motorLiftT");
        motorLiftB=hardwareMap.dcMotor.get("motorLiftB");
        motorLiftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters_IMU);

        timerOpMode = new ElapsedTime();
    }

    // Just here to make the code happy
    @Override
    public void runOpMode() throws InterruptedException {}

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
        motorCollectionL.setPower(power);
        motorCollectionR.setPower(-power);
    }



    /** ====================== AUTONOMOUS METHODS ======================== */

    /** Before you stress out... this isn't here to make you feel bad. But you WILL need it to
     * write the rest of autonomous! As you call the method, just replace all the variable names
     * with whatever you need the autonomous to accomplish!*/
    void encoderDriveMecanum(boolean isTest, double baseMotorPower, double distanceInInchesForward, double distancesInInchesStrafing){ //Distance in inches, x and y as direction values, both between 1- and 1
        // DOING EPIC MATH (Based on Jamari's TeleOp Driving code from last year)
        double distanceInInches = Math.hypot(distancesInInchesStrafing, distanceInInchesForward);
        double angleDirection = Math.atan2(distanceInInchesForward, distancesInInchesStrafing); // May need to catch an ArithmeticException here
        double xTargetDirection = Math.cos(angleDirection);
        double yTargetDirection = Math.sin(angleDirection);

        double drivingPower = baseMotorPower * Math.hypot(xTargetDirection, yTargetDirection);
        double wheelTrajectory = angleDirection - (Math.PI/4);

        // The mathematics above might be epic, but are they how we WANT them to be?
        while(isTest && !gamepad1.a) {
            telemetry.addData("distanceInInches", distanceInInches);
            telemetry.addData("angleHeading", Math.toDegrees(angleDirection));
            telemetry.addData("drivingPower", drivingPower);
            telemetry.addData("robotTrajectory", Math.toDegrees(wheelTrajectory));
            telemetry.addData("Looks Good?", "Press A if it's so before we crash and burn");
            telemetry.update();
        }

        // Determining speeds for each drive motor
        double desiredSpeedLF = (drivingPower * (Math.cos(wheelTrajectory)));
        double desiredSpeedLB = (drivingPower * (Math.sin(wheelTrajectory)));
        double desiredSpeedRF = -(drivingPower * (Math.sin(wheelTrajectory)));
        double desiredSpeedRB = -(drivingPower * (Math.cos(wheelTrajectory)));

        // Determining targets for each drive motor
        int newTargetLF = motorDriveLF.getCurrentPosition() + (int)(distanceInInches * desiredSpeedLF * COUNTS_PER_INCH);
        int newTargetLB = motorDriveLB.getCurrentPosition() + (int)(distanceInInches * desiredSpeedLB * COUNTS_PER_INCH);
        int newTargetRF = motorDriveRF.getCurrentPosition() + (int)(distanceInInches * desiredSpeedRF * COUNTS_PER_INCH);
        int newTargetRB = motorDriveRB.getCurrentPosition() + (int)(distanceInInches * desiredSpeedRB * COUNTS_PER_INCH);

        // Is the math behind the Speeds and Distances for each motor cool and good?
        while(isTest && !gamepad1.b) {
            telemetry.addData("motorDriveLF Speed", desiredSpeedLF);
            telemetry.addData("motorDriveLB Speed", desiredSpeedLB);
            telemetry.addData("motorDriveRF Speed", desiredSpeedRF);
            telemetry.addData("motorDriveRB Speed", desiredSpeedRB);
            telemetry.addData("motorDriveLF Target", newTargetLF);
            telemetry.addData("motorDriveLB Target", newTargetLB);
            telemetry.addData("motorDriveRF Target", newTargetRF);
            telemetry.addData("motorDriveRB Target", newTargetRB);
            telemetry.addData("Looks Good?", "Press B if it's so before we crash and burn");
            telemetry.update();
        }

        // Setting the determined targets to each drive motor
        motorDriveLF.setTargetPosition(newTargetLF);
        motorDriveLB.setTargetPosition(newTargetLB);
        motorDriveRF.setTargetPosition(newTargetRF);
        motorDriveRB.setTargetPosition(newTargetRB);

        // Setting each drive motor to RUN_TO_POSITION
        motorDriveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Power the drive train!
        setDrivePowerMotors(desiredSpeedLF, desiredSpeedLB, desiredSpeedRF, desiredSpeedRB);

        /* A loop to keep track of each motor's targets and positions as it runs.
         * To end once the motors hit their targets */
        while(opModeIsActive() &&
                (motorDriveLF.isBusy() || motorDriveLB.isBusy() || motorDriveRF.isBusy() || motorDriveRB.isBusy())) {
            telemetry.addData("motorDriveLF Target", newTargetLF);
            telemetry.addData("motorDriveLB Target", newTargetLB);
            telemetry.addData("motorDriveRF Target", newTargetRF);
            telemetry.addData("motorDriveRB Target", newTargetRB);
            telemetry.addData("motorDriveLF Current Position", motorDriveLF.getCurrentPosition());
            telemetry.addData("motorDriveLB Current Position", motorDriveLB.getCurrentPosition());
            telemetry.addData("motorDriveRF Current Position", motorDriveRF.getCurrentPosition());
            telemetry.addData("motorDriveRB Current Position", motorDriveRB.getCurrentPosition());
            telemetry.update();
        }

        // Shut off motors, because we have arrived, baby!
        setDrivePower(0);
    }
    /** Just an angle converter to use in case we ever need to turn to an absolute heading, instead
     * of a heading relative to the robot.*/
    public double absoluteHeading(double target) {
        target += 180;
        target += target > 360 ? -360 :
                target <   0 ?  360 : 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = -angles.firstAngle + 180;
        double degreesToTurn = target - currentHeading;
        degreesToTurn += degreesToTurn > 180  ? -360 :
                degreesToTurn < -180 ?  360 : 0;
        return degreesToTurn;
    }

    /** The IMU Turn we used last year. Not as complicated as the encoder drive, yet more
     * complicated at the same time. Enjoy.*/
    public void imuTurn(double degreesToTurn) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle + 180;
        double targetHeading = degreesToTurn + currentHeading;

        while(targetHeading > 360) targetHeading -= 360;

        while (Math.abs(degreesToTurn) > 2) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            double power = Range.clip(Math.signum(degreesToTurn) * (0.25 + (Math.abs(degreesToTurn) / 360)), -1, 1);
            setDrivePowerSides(power, -power);

            telemetry.addData("DegreesToTurn", degreesToTurn);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Current", currentHeading);
            telemetry.update();
        }
    }

    // <INSERT FOUNDATION SCORING METHOD HERE>

    public void driveToQuarryAndScoreStonesOnFoundation() {
        encoderDriveMecanum(false, 1, 0, -24); // Strafe left of the foundation
        encoderDriveMecanum(false, 1, 24, -24);
        encoderDriveMecanum(false, 1, 0, -24);
        imuTurn(135);
        setCollectionPower(1);
        encoderDriveMecanum(false, 0.7, -16, 0);
        encoderDriveMecanum(false, 0.7, 16, 0);
        setCollectionPower(0);
        imuTurn(45);
        encoderDriveMecanum(false, 1, 36, 0);
        encoderDriveMecanum(false, 0.7, 0, 12);
        // Scoring Goes Here
        if(30 - timerOpMode.seconds() > 12) {
            encoderDriveMecanum(false, 0.7, 0, -12);
            encoderDriveMecanum(false, 1, -36, 0);
            imuTurn(135);
            setCollectionPower(1);
            encoderDriveMecanum(false, 0.7, -16, 0);
            encoderDriveMecanum(false, 0.7, 16, 0);
            setCollectionPower(0);
            imuTurn(45);
            encoderDriveMecanum(false, 1, 36, 0);
            encoderDriveMecanum(false, 0.7, 0, 12);
            // Scoring Goes Here
        }
        encoderDriveMecanum(false, 0.7, 0, -12);
        encoderDriveMecanum(false, 1, -24, 0);
    }

    // <INSERT SKYBRIDGE METHOD HERE>

    /** ====================== TELEOP METHODS ======================== */

    // <INSERT DRIVER CONTROLS METHOD HERE (NOTE: remember that this will be updated continuously)>

    // <INSERT ANY OPERATOR CONTROLS METHOD HERE (NOTE: remember that this will be updated continuously)>
    // SPECIAL NOTE: Because of how sketchy the fourbar lift is, I plan on helping you out with fixing that. :D
}
