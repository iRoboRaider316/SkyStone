package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="The Robot Autonomous", group="LinearOpMode")
public class TherobotAutonomous extends LinearOpMode {
    public TherobotBase therobot;
    @Override
    public void runOpMode() throws InterruptedException {
        therobot = new TherobotBase(this);
        therobot.selection();
        waitForStart();
        therobot.timerOpMode.reset();
        therobot.timerCheck();
        if(therobot.pushingFoundation) {
            therobot.scoreFoundation();
            therobot.pushTheFoundation();
        }
        therobot.driveToSkybridge();
    }
    // See how short that was? :D
}