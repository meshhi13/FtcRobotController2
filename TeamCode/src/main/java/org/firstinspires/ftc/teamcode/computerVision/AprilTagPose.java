package org.firstinspires.ftc.teamcode.computerVision;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "April Tag Pose")
public class AprilTagPose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        AprilTagProcessor tagProcessor = initAprilTag();
        VisionPortal vision = initVisionPortal(tagProcessor);
        while (!isStopRequested() && opModeIsActive()){
            if (tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x: ", tag.ftcPose.x);
                telemetry.addData("y: ", tag.ftcPose.y);
                telemetry.addData("z: ", tag.ftcPose.z);
                telemetry.addData("roll: ", tag.ftcPose.roll);
                telemetry.addData("pitch: ", tag.ftcPose.pitch);
                telemetry.addData("yaw: ", tag.ftcPose.yaw);
                telemetry.addData("x: ", tag.ftcPose.elevation);
                telemetry.addData("bearing: ", tag.ftcPose.bearing);
                telemetry.addData("elevation: ", tag.ftcPose.elevation);
            }
            else {
                telemetry.addData("x: ", "null");
                telemetry.addData("y: ", "null");
                telemetry.addData("z: ", "null");
                telemetry.addData("x: ", "null");
                telemetry.addData("roll: ", "null");
                telemetry.addData("pitch: ", "null");
                telemetry.addData("yaw: ", "null");
                telemetry.addData("bearing: ", "null");
                telemetry.addData("elevation: ", "null");
            }
            telemetry.update();
        }
    }
    public AprilTagProcessor initAprilTag() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        return tagProcessor;
    }
    public VisionPortal initVisionPortal(AprilTagProcessor tagProcessor) {
        VisionPortal vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 448))
                .addProcessor(tagProcessor)
                .build();

        return vision;
    }
}
