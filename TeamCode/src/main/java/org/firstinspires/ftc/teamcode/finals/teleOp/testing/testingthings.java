package org.firstinspires.ftc.teamcode.finals.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.finals.teleOp.TeleOpMethods;

@TeleOp (name = "testing things")
public class testingthings extends TeleOpMethods {
    TouchSensor touch1;
    TouchSensor touch2;
    Servo testServo;
    DcMotor hanger;
    CRServo testCRServo;
    DcMotor droneLauncher;
    double behind;
    DcMotor slideMotor;
    TouchSensor slideLimit;

    @Override
    public void init() {
        hanger = hardwareMap.dcMotor.get("hanger");
    }
    @Override
    public void loop() {
        getController();

        //Only allow hanger to move if not at the extrema, and the limits are not touched.
        if (g1_rightstick_y < 0 && !touch1.isPressed()){
            hanger.setPower(-g1_rightstick_y);
        }
        if (g1_rightstick_y > 0 && !touch2.isPressed()){
            hanger.setPower(-g1_rightstick_y);
        }

        telemetry.addData("Hanger Power: ", hanger.getPower());
        telemetry.update();
    }
}
