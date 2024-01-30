package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drone Launcher")
public class DroneLauncher extends OpMode {
    Servo launchServo;

    public void init() {
        launchServo = hardwareMap.servo.get("launchServo");
    }

    public void loop() {

        if (gamepad2.dpad_down){
            launchServo.setPosition(-1);
        }
        if (gamepad2.dpad_up){
            launchServo.setPosition(1);
        }
    }
}