package org.firstinspires.ftc.teamcode.finals.teleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {
    //Driving Variables
    public static final double driveSpeed = 0.66;
    public static final double fastSpeed = 1.0;
    public static final double slowSpeed = 0.25;
    public static double finalSlowmode = 0.0;
    public static boolean slowMode = true;
    public static int gyroAdj = 0;

    //Automated time variables
    double current1 = Double.MAX_VALUE;
    double current2 = Double.MAX_VALUE;
    double current3 = Double.MAX_VALUE;
    double current4 = Double.MAX_VALUE;
    double current5 = Double.MAX_VALUE;

    //Lifting and Servo Variables
    int liftPos = 0;
    int liftPosAdj = 0;
    double liftPower = 0.0;
    double clawPos = 0.0;
    double servoRot = 0.0;
    double outtakeRot = -1.0;
    boolean startHanging = false;

    public void getController() {
        //Gamepad joysticks
        g1_leftstick_x = gamepad1.left_stick_x;
        g2_leftstick_x = gamepad2.left_stick_x;

        g1_leftstick_y = gamepad1.left_stick_y;
        g2_leftstick_y = gamepad2.left_stick_y;

        g1_rightstick_x = gamepad1.right_stick_x;
        g2_rightstick_x = gamepad2.right_stick_x;

        g1_rightstick_y = gamepad1.right_stick_y;
        g2_rightstick_y = gamepad2.right_stick_y;


        //Directional pad buttons
        g1_dpad_down = gamepad1.dpad_down;
        g1_dpad_up = gamepad1.dpad_up;
        g1_dpad_right = gamepad1.dpad_right;
        g1_dpad_left = gamepad1.dpad_left;
        g2_dpad_down = gamepad2.dpad_down;
        g2_dpad_up = gamepad2.dpad_up;
        g2_dpad_right = gamepad2.dpad_right;
        g2_dpad_left = gamepad2.dpad_left;

        //Gamepad buttons
        g1_a = gamepad1.a;
        g1_b = gamepad1.b;
        g1_x = gamepad1.x;
        g1_y = gamepad1.y;
        g2_a = gamepad2.a;
        g2_b = gamepad2.b;
        g2_x = gamepad2.x;
        g2_y = gamepad2.y;

        //Gamepad bumpers
        g1_right_bumper = gamepad1.right_bumper;
        g1_left_bumper = gamepad1.left_bumper;
        g2_right_bumper = gamepad2.right_bumper;
        g2_left_bumper = gamepad2.left_bumper;

        //Gamepad triggers
        g1_right_trigger = gamepad1.right_trigger;
        g1_left_trigger = gamepad1.left_trigger;
        g2_right_trigger = gamepad2.right_trigger;
        g2_left_trigger = gamepad2.left_trigger;
    }

    public void scoringPoints(){

        if (g1_dpad_left){
            launchServo.setPosition(-1);

        }
        if (g1_dpad_right){
            launchServo.setPosition(1);

        }

        if (g2_a){
            current2 = getRuntime();
            clawPos = 1.0;
        }

        if (getRuntime() > current2 + .75){
            current3 = getRuntime();
            servoRot = 0.601;
            current2 = Double.MAX_VALUE;
        }

        if (getRuntime() > current3 + .75){
            current4 = getRuntime();
            clawPos = 0.0;
            current3 = Double.MAX_VALUE;
        }

        if (getRuntime() > current4 + .75){
            current5 = getRuntime();
            servoRot = 0.0;
            current4 = Double.MAX_VALUE;
        }

//        if (getRuntime() >  current5 + 1.0){
//            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setTargetPosition(1000);
//            slideMotor.setPower(1.0);
//        }

        //Open and close claw
        if (g2_right_bumper){
            clawPos = 1.0;
        }
        if (g2_left_bumper){
            clawPos = 0.0;
        }

        //Limit servo rotations for claw rotation
        if (servoRot > 1){
            servoRot = 1;
        }
        if (servoRot < 0){
            servoRot = 0;
        }

        //Rotate servo using joystick
        servoRot += g2_right_trigger * 0.02;
        servoRot -= g2_left_trigger * 0.02;

        //Set servo position and give time to run
        if (g2_x){
            current1 = getRuntime();
            outtakeRot = 1;
        }

        //Rotate back after, so one click is all that's needed
        if (getRuntime() > current1 + .75){
            outtakeRot = -1;

            current1 = Double.MAX_VALUE;
        }

        //Limt servo rotations for outtake servo
        if (outtakeRot > 1){
            outtakeRot = 1;
        }
        if (outtakeRot < -1){
            outtakeRot = -1;
        }

        if (g2_dpad_down){
            servoRot = 0.0;
        }
        if (g2_dpad_left){
            servoRot = 0.06;
        }
        if (g2_dpad_up){
            servoRot = 0.12;
        }

        //Rotate servo using joystick
        outtakeRot += -g2_rightstick_y * 0.04;

        //Only allow hanger to move if not at the extrema, and the limits are not touched.

        if (g1_dpad_up){
            startHanging = true;
        }
        if (g1_dpad_down){
            startHanging = false;
            if (!touch1.isPressed()){
                hanger.setPower(-1.0);
            }
        }

        if (startHanging && !touch2.isPressed()) { hanger.setPower(1.0); }
        else if (g1_dpad_down && !touch1.isPressed()) { hanger.setPower(-1.0); }
        else { hanger.setPower(0); }

        // Move the LIFT (Motor Up/Down)
        liftPos = slideMotor.getCurrentPosition() - liftPosAdj;

        if (slideLimit.isPressed() || g2_left_bumper){
            liftPosAdj = slideMotor.getCurrentPosition();
        }
        if (g2_leftstick_y > 0 && !slideLimit.isPressed()) {
            liftPower = g2_leftstick_y;
        }
        else if (g2_leftstick_y < 0.0 && liftPos < 4020){
            liftPower = g2_leftstick_y;
        }
        else { liftPower = 0; }

        slideMotor.setPower(-liftPower);
        outtakeServo.setPosition(outtakeRot);
        clawRotateServo.setPosition(servoRot);
        clawServo.setPosition(clawPos);
    }

    public void mecanumDrive(){

        //Setting boolean hold
        if(g1_right_bumper) {
            //Slowmode
            finalSlowmode = slowSpeed;

        } else if (g1_left_bumper) {
            //Fastmode
            finalSlowmode = fastSpeed;
        } else {
            //Regular
            finalSlowmode = driveSpeed;
        }

        if (g1_y){
            imu.resetYaw();
        }


        orientation = imu.getRobotYawPitchRollAngles();
        mecanumDrive.driveFieldCentric(g1_leftstick_x * finalSlowmode, -g1_leftstick_y * finalSlowmode, gamepad1.right_stick_x * finalSlowmode, orientation.getYaw(AngleUnit.DEGREES));
    }

    public void addTelemetryToDriverStation() {
//        telemetry.addData("Distance: ", "LEFT: " + left.getDistance(DistanceUnit.INCH) + " || RIGHT: " + right.getDistance(DistanceUnit.INCH) + " || BACK: " + back.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rumbling: ", gamepad1.isRumbling() ? "YES" : "NO" );
        telemetry.addData("Slide Motor: ", liftPos);
        telemetry.addData("GPad1 LY: ", g2_leftstick_y);
        telemetry.addData("Touch Bottom: ", touch1.isPressed());
        telemetry.addData("Touch Top: ", touch2.isPressed());
        telemetry.addData("Roll: ", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Pitch: ", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Outtake Servo: ", outtakeServo.getPosition());
        telemetry.addData("Claw Servo: ", clawServo.getPosition());
        telemetry.addData("Rotate Servo: ", clawRotateServo.getPosition());

        telemetry.update();
    }
}


