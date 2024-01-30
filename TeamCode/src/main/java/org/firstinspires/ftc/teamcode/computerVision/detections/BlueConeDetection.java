package org.firstinspires.ftc.teamcode.computerVision.detections;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous(name = "Blue Cone Detection")
public class BlueConeDetection extends LinearOpMode {
    OpenCvCamera camera;
    ObserverPipeline pipeline;
    private static final int CAMERA_WIDTH = 800; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 448; // height of wanted camera solution
    String position;



    @Override
    public void runOpMode() {
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        while (!isStarted()){
            telemetry.addData("POSITION: ", pipeline.getPosition());
            telemetry.update();
        }

        position = pipeline.getPosition();
        camera.stopStreaming();
        camera.closeCameraDevice();

        waitForStart();

        AprilTagProcessor tagProcessor = initAprilTag();
        VisionPortal vision = initVisionPortal(tagProcessor);

        while (opModeIsActive()){
            if (tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                try { if (position.equals("LEFT"))   { tag = tagProcessor.getDetections().get(0); } }
                catch (Exception ignored){ ; }
                try { if (position.equals("CENTER")) { tag = tagProcessor.getDetections().get(1); } }
                catch (Exception ignored){ ; }
                try { if (position.equals("RIGHT"))  { tag = tagProcessor.getDetections().get(2);} }
                catch (Exception ignored){ ; }

                telemetry.addData("id: ", tag.id);
                telemetry.addData("x: ", tag.ftcPose.x);
                telemetry.addData("y: ", tag.ftcPose.y);
                telemetry.addData("z: ", tag.ftcPose.z);
                telemetry.addData("roll: ", tag.ftcPose.roll);
                telemetry.addData("pitch: ", tag.ftcPose.pitch);
                telemetry.addData("yaw: ", tag.ftcPose.yaw);
                telemetry.addData("bearing: ", tag.ftcPose.bearing);
                telemetry.addData("elevation: ", tag.ftcPose.elevation);
            }
            else {
                telemetry.addData("id: ", "null");
                telemetry.addData("x: ", "null");
                telemetry.addData("y: ", "null");
                telemetry.addData("z: ", "null");
                telemetry.addData("roll: ", "null");
                telemetry.addData("pitch: ", "null");
                telemetry.addData("yaw: ", "null");
                telemetry.addData("bearing: ", "null");
                telemetry.addData("elevation: ", "null");
            }
            telemetry.update();
        }
    }

    private VisionPortal initVisionPortal(AprilTagProcessor tagProcessor) {
        VisionPortal vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 448))
                .addProcessor(tagProcessor)
                .build();

        return vision;
    }

    private AprilTagProcessor initAprilTag() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        return tagProcessor;
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );
        pipeline = new ObserverPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
    class ObserverPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat rightCrop;
        Mat leftCrop;
        Mat centerCrop;
        double leftAvgFin;
        double rightAvgFin;
        double centerAvgFin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
        Scalar foundColor = new Scalar(0.0, 0.0, 255.0);

        String whereIsProp;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


            Rect leftRect = new Rect(0, 0, 266, 448);
            Rect centerRect = new Rect(267, 0, 267, 448);
            Rect rightRect = new Rect(534, 0, 266, 448);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, centerRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            centerCrop = YCbCr.submat(centerRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(centerCrop, centerCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar rightAvg = Core.mean(rightCrop);
            Scalar centerAvg = Core.mean(centerCrop);
            Scalar leftAvg = Core.mean(leftCrop);

            leftAvgFin = leftAvg.val[0];
            rightAvgFin = rightAvg.val[0];
            centerAvgFin = centerAvg.val[0];

            if (leftAvgFin > centerAvgFin && leftAvgFin > rightAvgFin) {
                whereIsProp = "LEFT";
                Imgproc.rectangle(output, leftRect, foundColor, 2);
            }
            else if (centerAvgFin > rightAvgFin && centerAvgFin > leftAvgFin) {
                whereIsProp = "CENTER";
                Imgproc.rectangle(output, centerRect, foundColor, 2);
            }
            else {
                whereIsProp = "RIGHT";
                Imgproc.rectangle(output, rightRect, foundColor, 2);
            }


            return output;
        }
        public String getPosition(){
            return whereIsProp;
        }
    }

}


