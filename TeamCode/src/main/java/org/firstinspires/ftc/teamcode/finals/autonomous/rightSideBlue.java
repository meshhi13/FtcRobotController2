package org.firstinspires.ftc.teamcode.finals.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RIGHT SIDE BLUE")
public class rightSideBlue extends AutoMethods {
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

        Pose2d startPose = new Pose2d(12.00, 72.00, Math.toRadians(270.00));
        drive.setPoseEstimate(startPose);
        position = bluePipeline.getPosition();
        camera.stopStreaming();
        camera.closeCameraDevice();

        waitForStart();

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

        rotateElapsed(0.4, .5);
        driveForward(19);
        strafeLeft(26);
        if (position != PropPosition.LEFT) {
            strafeLeft(1);
            driveBackward(1);
        }
        rotateElapsed(.118, .5);
        clawElapsed(1.0, 1);
        rotateElapsed(1, .5);
        clawElapsed(0, .3);
        driveBackward(106);


        if (position == PropPosition.RIGHT){
            strafeRight(21);
        } else if (position == PropPosition.CENTER) {
            strafeRight(27);
        } else {
            strafeRight(33);
        }

        if (back.getDistance(DistanceUnit.INCH) > 10){
            driveBackward(8);
        }
        else {
            driveBackward(back.getDistance(DistanceUnit.INCH) - 1.5);
        }
        rotateElapsed(0.4, .5);
        outtakeElapsed(.15, .5);
        liftArm(2908, 1, 1);
        outtakeElapsed(1, 1);
        outtakeElapsed(0, .5);

        driveForward(1);
        resetArm(1, 1);

        while (opModeIsActive()) {
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
            stop();
        }
    }
}