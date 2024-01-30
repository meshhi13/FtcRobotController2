package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MecDrive")
public class MecDrive extends OpMode {
    private Motor rightFront, leftFront, rightBack, leftBack;
    private GamepadEx driverOp, gunnerOp;
    private RevIMU imu;
    private MecanumDrive mecanum;

    private double heading;
    private double secondHeading = 0;



    @Override
    public void init() {
        // Change the motors to the ones that we have
        rightFront = new Motor(hardwareMap, "rightFront");
        leftFront = new Motor(hardwareMap, "leftFront");
        rightBack = new Motor(hardwareMap, "rightBack");
        leftBack = new Motor(hardwareMap, "leftBack");

        rightFront.resetEncoder();
        leftFront.resetEncoder();
        rightBack.resetEncoder();
        leftBack.resetEncoder();

        leftFront.setInverted(true);
        rightFront.setInverted(false);
        leftBack.setInverted(false);
        rightBack.setInverted(false);

        driverOp = new GamepadEx(gamepad1);
        gunnerOp = new GamepadEx(gamepad2);
        imu = new RevIMU(hardwareMap);
        imu.init();
        mecanum = new MecanumDrive(rightFront, leftFront, rightBack, leftBack);
    }

    @Override
    public void loop() {
        heading = imu.getRotation2d().getDegrees() - secondHeading;
        if (driverOp.getButton(GamepadKeys.Button.A)){

            secondHeading = imu.getRotation2d().getDegrees();
        }

        mecanum.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
               imu.getRotation2d().getDegrees()
        );

        telemetry.addData("Angle: ", imu.getRotation2d().getDegrees());
        telemetry.addData("Button Pressed: ", driverOp.getButton(GamepadKeys.Button.A));
        telemetry.addData("Right Front: ", rightFront.getCurrentPosition());
        telemetry.addData("Left Front: ", leftFront.getCurrentPosition());
        telemetry.addData("Right Back: ", rightBack.getCurrentPosition());
        telemetry.addData("Left Back: ", leftBack.getCurrentPosition());
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.update();
    }
}

