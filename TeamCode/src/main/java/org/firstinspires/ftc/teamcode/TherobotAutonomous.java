package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TherobotAutonomous extends LinearOpMode {

    public TherobotBase therobotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        therobotBase = new TherobotBase(this);
        waitForStart();
        // Right now this is just a test class for mecanum driving so I can refine the mecanum drive ASAP.
        therobotBase.imuTurn(90);
        therobotBase.encoderDriveMecanum(true, 1, 24, 0);
        therobotBase.encoderDriveMecanum(true, 1, 0, 24);
        therobotBase.encoderDriveMecanum(true, 1, -48, 0);
        therobotBase.encoderDriveMecanum(true, 0.5, 24, -24);
    }

    // See how short that was? :D
}
