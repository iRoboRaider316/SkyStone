package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TherobotAutonomous extends LinearOpMode {

    public TherobotBase therobot;

    @Override
    public void runOpMode() throws InterruptedException {
        therobot = new TherobotBase(this);
        waitForStart();
        // Right now this is just a test class for mecanum driving so I can refine the mecanum drive ASAP.
        therobot.imuTurn(90);
        therobot.encoderDriveMecanum(true, 1, 24, 0);
        therobot.encoderDriveMecanum(true, 1, 0, 24);
        therobot.encoderDriveMecanum(true, 1, -48, 0);
        therobot.encoderDriveMecanum(true, 0.5, 24, -24);
    }

    // See how short that was? :D
}
