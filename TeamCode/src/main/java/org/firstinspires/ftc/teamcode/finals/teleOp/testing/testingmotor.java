package org.firstinspires.ftc.teamcode.finals.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.finals.teleOp.TeleOpMethods;

@TeleOp (name = "testing motor")
public class testingmotor extends TeleOpMethods {
    DcMotor slideMotor;
    TouchSensor slideLimit;
    double servoRot = 0;
    Servo outtakeServo;
    double current1, liftPower;
    int liftPos, liftPosAdj;
    @Override
    public void init() {
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLimit = hardwareMap.touchSensor.get("slideLimit");
        outtakeServo = hardwareMap.servo.get("outtakeServo");
        outtakeServo.setPosition(0.15);
    }
    @Override
    public void loop() {
        getController();

        liftPower = 0;
        liftPos = slideMotor.getCurrentPosition() - liftPosAdj;

        if (g2_left_bumper){
            liftPosAdj = slideMotor.getCurrentPosition();
        }


        if (g2_rightstick_y <= 0.0 && liftPos < 8700){
            liftPower = g2_rightstick_y * 1.0;  //UP
        }
        if(g2_rightstick_y > 0.0){
            liftPower = g2_rightstick_y * 1.0;  // DOWN
        }

        slideMotor.setPower(-liftPower);

        //Set servo position and give time to run
        if (g2_x){
            current1 = getRuntime();
            outtakeServo.setPosition(0.6);
        }

        //Rotate back after, so one click is all that's needed
        if (getRuntime() > current1 + 1.5){
            outtakeServo.setPosition(0.15);
            current1 = Double.MAX_VALUE;
        }

        if (servoRot > 1){
            servoRot = 1;
        }
        if (servoRot < 0){
            servoRot = 0;
        }

        //Rotate servo using joystick
        servoRot += -g2_leftstick_y * 0.005;
        outtakeServo.setPosition(servoRot);

        telemetry.addData("SlideMotor: ", slideMotor.getCurrentPosition());
        telemetry.addData("SlideLimit: ", slideLimit.isPressed());
        telemetry.addData("SlideLiftPosAdj: ", liftPosAdj);
        telemetry.addData("OuttakeServo: ", outtakeServo.getPosition());

        telemetry.update();
    }
}
