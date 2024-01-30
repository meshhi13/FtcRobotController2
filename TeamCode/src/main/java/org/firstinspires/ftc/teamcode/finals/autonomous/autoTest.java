package org.firstinspires.ftc.teamcode.finals.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Test")
public class autoTest extends AutoMethods {
    @Override

    public void runOpMode() throws InterruptedException {
        initRobot();
        clawElapsed(1.0, 1);
        waitForStart();
        driveForward(26);
        rotateElapsed(0.0, 1);
        sleep(1000);
        clawElapsed(0, 1);
        rotateElapsed(0.67, 1);

    }
}
