package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Mando Autonomous", group="")
public class SteveAutonomous extends LinearOpMode {

    SteveBase steve;

    @Override
    public void runOpMode() throws InterruptedException {
        steve = new SteveBase(this);
        steve.selection();
        waitForStart();
        steve.timerOpMode.reset();
        if(steve.pushingFoundation) {
            steve.grabFoundation();
            steve.turnAndScoreFoundation();
        }
        steve.driveToSkybridge();
        steve.waitForEnd();
        steve.storeHeading();

    }
}
