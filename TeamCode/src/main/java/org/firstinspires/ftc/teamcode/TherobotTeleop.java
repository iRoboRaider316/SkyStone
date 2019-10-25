package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    }
}
