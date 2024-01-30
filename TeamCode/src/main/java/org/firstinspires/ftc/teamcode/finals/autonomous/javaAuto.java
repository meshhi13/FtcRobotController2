package org.firstinspires.ftc.teamcode.finals.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Java Auto Test")
public class javaAuto extends AutoMethods {
    @Override

    public void runOpMode() throws InterruptedException {
        initRobot();
        clawElapsed(1.0, 1);
        waitForStart();
        rotateElapsed(0.0, 1);
        clawElapsed(0, 1);
        rotateElapsed(0.67, 1);
        liftArm(1000, 1, 2);
        resetArm(1,  2);
    }
}


