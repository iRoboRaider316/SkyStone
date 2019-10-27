package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="The Robot TeleOp", group="OpMode")
public class TherobotTeleop extends OpMode {

    TherobotBase therobot;

    @Override
    public void init() {
        therobot = new TherobotBase(this);
    }

    @Override
    public void start() {
        therobot.timerOpMode.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Yay!", therobot.timerOpMode.seconds());
    }
}
