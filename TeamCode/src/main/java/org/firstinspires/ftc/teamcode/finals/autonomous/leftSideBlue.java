package org.firstinspires.ftc.teamcode.finals.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "LEFT SIDE BLUE")
public class leftSideBlue extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        clawElapsed(1.0, 1);
        blueInitOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        while (!isStarted()){
            telemetry.addData("POSITION: ", bluePipeline.getPosition());
            telemetry.update();
        }

        Pose2d startPose = new Pose2d(-36.00, 72.00, Math.toRadians(270.00));
        drive.setPoseEstimate(startPose);
        position = bluePipeline.getPosition();
        camera.stopStreaming();
        camera.closeCameraDevice();

        waitForStart();
        driveForward(22);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        if (position == PropPosition.RIGHT){
            turnWithDegrees(-45);
            driveForward(10);
            driveBackward(6);
            clawRotateServo.setPosition(0);
            sleep(1000);
            clawServo.setPosition(0.0);
            sleep(1000);
            clawRotateServo.setPosition(0.4);
            driveBackward(4);
            turnWithDegrees(45);
        } else if (position == PropPosition.CENTER){
            driveForward(14);
            driveBackward(12);
            clawRotateServo.setPosition(0);
            sleep(1000);
            clawServo.setPosition(0.0);
            sleep(1000);
            clawRotateServo.setPosition(0.4);
            driveBackward(2);
        } else {
            turnWithDegrees(45);
            driveForward(10);
            driveBackward(6);
            clawRotateServo.setPosition(0);
            sleep(1000);
            clawServo.setPosition(0.0);
            sleep(1000);
            clawRotateServo.setPosition(0.4);
            driveBackward(4);
            turnWithDegrees(-45);
        }

        rotateElapsed(0.2, .5);
        driveBackward(2);
        strafeLeft(32);
        turnWithDegrees(-90);

        if (position == PropPosition.LEFT){
            strafeRight(1);
        } else if (position == PropPosition.CENTER) {
            strafeLeft(6);
        } else {
            strafeLeft(10);
        }

        if (back.getDistance(DistanceUnit.INCH) > 10){
            driveBackward(8);
        }
        else {
            driveBackward(back.getDistance(DistanceUnit.INCH) - 1.5);
        }

        outtakeElapsed(.15, .5);
        liftArm(2208, 1, 1.5);
        outtakeElapsed(1, 1);
        outtakeElapsed(0, .5);
        driveForward(2);

        if (position == PropPosition.LEFT){
            strafeRight(3);
        } else if (position == PropPosition.CENTER) {
            strafeRight(8);
        } else {
            strafeRight(14);
        }

        strafeRight(18);
        driveBackward(back.getDistance(DistanceUnit.INCH) - 4);
        resetArm(1, 2);

        while (opModeIsActive()){
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
            stop();
        }
    }
}