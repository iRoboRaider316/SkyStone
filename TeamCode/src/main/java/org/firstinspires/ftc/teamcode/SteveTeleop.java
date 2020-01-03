package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Steve TeleOp", group="OpMode")
public class SteveTeleop extends OpMode {

    SteveBase steve;

    @Override
    public void init() {
        steve = new SteveBase(this);
    }

    @Override
    public void loop() {
        steve.updateDriveTrain();
    }
}
