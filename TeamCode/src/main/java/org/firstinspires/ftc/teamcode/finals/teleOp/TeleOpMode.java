package org.firstinspires.ftc.teamcode.finals.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Centerstage Drive 1.0.1")
public class TeleOpMode extends TeleOpMethods {

    public void loop(){
        getController();
        mecanumDrive();
        scoringPoints();
        addTelemetryToDriverStation();
    }
}

