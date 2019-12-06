package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="The Robot TeleOp", group="OpMode")
public class TherobotTeleop extends OpMode {

    TherobotBase therobot;

    @Override
    public void init() {
        therobot = new TherobotBase(this);
        therobot.resetEncoders();
        therobot.runWithoutEncoders();
    }

    @Override
    public void start() {
        therobot.timerOpMode.reset();
    }

    @Override
    public void loop() {
        therobot.mecanumDrive();
        therobot.controlCollection();
        therobot.controlDeliveryArm();
        therobot.controlFoundationServos(gamepad1.right_bumper, gamepad1.left_bumper);
        therobot.controlServoClaw();
        therobot.controlServoGrabberSwivel(gamepad2.b, gamepad2.x);
        therobot.yeetCapstone();

        telemetry.addData("TeleOp is Active", therobot.timerOpMode.seconds());
        telemetry.update();
    }
}