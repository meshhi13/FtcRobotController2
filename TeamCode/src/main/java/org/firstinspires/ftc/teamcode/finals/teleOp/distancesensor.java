package org.firstinspires.ftc.teamcode.finals.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "Distance Test")
public class distancesensor extends OpMode {
    DistanceSensor distance;
    @Override
    public void init() {

        distance = hardwareMap.get(DistanceSensor.class, "distanceSensor");

    }
    public void loop() {
    double distanceInInches = distance.getDistance(DistanceUnit.INCH);

    telemetry.addData("Distance (inches)", distanceInInches);
    telemetry.update();

    }
}


