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
            driveForward(22);
            turnWithDegrees(-45);
            driveForward(8);
            driveBackward(5);
            rotateElapsed(0.1, 1);
            clawElapsed(0, 1);
            driveBackward(3);
            turnWithDegrees(-45);
        } else if (position == PropPosition.CENTER){
            driveForward(30);
            driveBackward(6);
            rotateElapsed(0.1, 1);
            clawElapsed(0, 1);
            driveBackward(2);
            turnWithDegrees(-90);
        } else {
            driveForward(22);
            turnWithDegrees(45);
            driveForward(8);
            driveBackward(5);
            rotateElapsed(0.1, 1);
            clawElapsed(0, 1);
            driveBackward(3);
            turnWithDegrees(-135);
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