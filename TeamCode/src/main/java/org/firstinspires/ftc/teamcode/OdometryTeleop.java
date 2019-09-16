package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Disabled
@TeleOp(name="Odometry Test",group="OpMode")
public class OdometryTeleop extends OpMode {

    double lTick = 0;
    double rTick = 0;
    double lTickPrev = 0;
    double rTickPrev = 0;
    double xPos = 0;
    double yPos = 0;
    double xPosPrev = 0;
    double yPosPrev = 0;
    double heading;
    private DcMotor motorDriveL, motorDriveR;         // Drive Train Motors
    private BNO055IMU imu;                  // IMU Gyro itself
    private Orientation angles;             // IMU Gyro's Orienting

    @Override
    public void init() {
        motorDriveR = hardwareMap.dcMotor.get("motorDriveR");
        motorDriveR.setPower(0);
        motorDriveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveL = hardwareMap.dcMotor.get("motorDriveL");
        motorDriveL.setPower(0);
        motorDriveL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveL.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters_IMU);

        motorDriveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        motorDriveL.setPower(gamepad1.left_stick_y / 4);
        motorDriveR.setPower(gamepad1.right_stick_y / 4);

        lTick = -(motorDriveL.getCurrentPosition() - lTickPrev);
        rTick = -(motorDriveR.getCurrentPosition() - rTickPrev);

        lTickPrev = motorDriveL.getCurrentPosition();
        rTickPrev = motorDriveR.getCurrentPosition();

        double lDist = 7 * Math.PI * (lTick / 288);
        double rDist = 7 * Math.PI * (rTick / 288);
        double cDist = (lDist + rDist) / 2;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

        xPos = xPosPrev + (cDist * Math.cos(Math.toRadians(heading)));
        yPos = yPosPrev + (cDist * Math.sin(Math.toRadians(heading)));

        xPosPrev = xPos;
        yPosPrev = yPos;

        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.addData("heading", heading);
        telemetry.addData("leftPos", motorDriveL.getCurrentPosition());
        telemetry.addData("rightPos", motorDriveR.getCurrentPosition());
        telemetry.update();

    }
}
