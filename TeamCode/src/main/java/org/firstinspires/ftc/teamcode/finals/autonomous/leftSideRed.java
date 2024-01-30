package org.firstinspires.ftc.teamcode.finals.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LEFT SIDE RED")
public class leftSideRed extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        clawElapsed(1.0, 1);
        redInitOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        while (!isStarted()){
            telemetry.addData("POSITION: ", redPipeline.getPosition());
            telemetry.update();
        }

        Pose2d startPose = new Pose2d(-36, -72, 90);
        drive.setPoseEstimate(startPose);
        position = redPipeline.getPosition();
        camera.stopStreaming();
        camera.closeCameraDevice();

        waitForStart();
        driveForward(22);

        if (position == PropPosition.RIGHT){
            turnWithDegrees(-45);
            driveForward(10);
            driveBackward(8);
            clawRotateServo.setPosition(0);
            sleep(1000);
            clawServo.setPosition(0.0);
            sleep(1000);
            clawRotateServo.setPosition(0.4);
            driveBackward(2);
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
            driveBackward(8);
            clawRotateServo.setPosition(0);
            sleep(1000);
            clawServo.setPosition(0.0);
            sleep(1000);
            clawRotateServo.setPosition(0.4);
            driveBackward(2);
            turnWithDegrees(-45);
        }

        while (opModeIsActive()){
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
            stop();
        }
    }
}