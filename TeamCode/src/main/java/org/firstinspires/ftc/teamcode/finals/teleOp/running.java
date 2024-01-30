package org.firstinspires.ftc.teamcode.finals.teleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Running")
public class running extends OpMode {
    MecanumDrive mecanumDrive;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    //Create the gyroscope
    public IMU imu;
    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;
    @Override
    public void init() {
        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFront = new Motor(hardwareMap, "LF");
        rightFront = new Motor(hardwareMap, "RF");
        leftBack = new Motor(hardwareMap, "LB");
        rightBack = new Motor(hardwareMap, "RB");

        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();

        mecanumDrive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, orientation.getYaw(AngleUnit.DEGREES));
    }
}
