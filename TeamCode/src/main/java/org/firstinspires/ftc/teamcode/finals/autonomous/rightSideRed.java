package org.firstinspires.ftc.teamcode.finals.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RIGHT SIDE RED")
public class rightSideRed extends AutoMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        redInitOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        FtcDashboard.getInstance().startCameraStream(camera, 30);
        initRobot();
        clawElapsed(1.0, 1);
        while (!isStarted()) {
            telemetry.addData("POSITION: ", redPipeline.getPosition());
            telemetry.update();
        }
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

        rotateElapsed(0.2, 1);
        driveBackward(2);
        strafeRight(34);
        turnWithDegrees(90);

        if (position == PropPosition.RIGHT){
            strafeRight(2);
        } else if (position == PropPosition.CENTER) {
            strafeRight(6);
        } else {
            strafeRight(14);
        }

        if (back.getDistance(DistanceUnit.INCH) > 10){
            driveBackward(8);
        }
        else {
            driveBackward(back.getDistance(DistanceUnit.INCH) - 1.5);
        }

        outtakeElapsed(.15, 1);
        liftArm(2208, 1, 2);
        outtakeElapsed(1, 1);
        outtakeElapsed(0, 1);

        driveForward(2);

        if (position == PropPosition.RIGHT){
            strafeLeft(3);
        } else if (position == PropPosition.CENTER) {
            strafeLeft(8);
        } else {
            strafeLeft(14);

        }

        strafeLeft(18);
        driveBackward(back.getDistance(DistanceUnit.INCH) - 4);
        resetArm(1, 2);

        while (opModeIsActive()) {
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
            stop();
        }
    }
}