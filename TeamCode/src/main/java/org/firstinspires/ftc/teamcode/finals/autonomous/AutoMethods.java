package org.firstinspires.ftc.teamcode.finals.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class AutoMethods extends LinearOpMode {
    //Create servos
    Servo launchServo;
    Servo clawServo;
    Servo clawRotateServo;
    Servo outtakeServo;

    //Create the magnet sensors
    TouchSensor touch1;
    TouchSensor touch2;
    TouchSensor slideLimit;

    //Create distance sensors
    DistanceSensor left;
    DistanceSensor right;
    DistanceSensor back;

    //Create the other motors
    DcMotor hanger;
    DcMotor slideMotor;

    //Motors using roadrunner
    SampleMecanumDrive drive;

    //Camera set-up
    OpenCvCamera camera;

    //Pick observer pipeline
    RedObserverPipeline redPipeline = new RedObserverPipeline();
    BlueObserverPipeline bluePipeline = new BlueObserverPipeline();

    //Create the gyroscope
    public IMU imu;
    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;

    public ElapsedTime runtime;

    //String for position
    public enum PropPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    PropPosition position = PropPosition.CENTER;


    private static final int CAMERA_WIDTH = 800; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 448; // height of wanted camera resolution

    public void initRobot() {
        drive = new SampleMecanumDrive(hardwareMap);
        runtime = new ElapsedTime();

        //Set up the other motors
        hanger = hardwareMap.dcMotor.get("hanger");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");

        //Set up the servos
        launchServo = hardwareMap.servo.get("launchServo");
        clawServo = hardwareMap.servo.get("clawServo");
        clawRotateServo = hardwareMap.get(Servo.class, "clawRotateServo");
        outtakeServo = hardwareMap.servo.get("outtakeServo");

        //Set up the sensors
        touch1 = hardwareMap.touchSensor.get("hangerLimitBottom");
        touch2 = hardwareMap.touchSensor.get("hangerLimitTop");
        slideLimit = hardwareMap.touchSensor.get("slideLimit");

        //Set up distance sensors
        left = hardwareMap.get(DistanceSensor.class, "left");
        right = hardwareMap.get(DistanceSensor.class, "right");
        back = hardwareMap.get(DistanceSensor.class, "back");


        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        launchServo.setPosition(-1);
    }

    public void driveForward(double distance) {
        drive.setPoseEstimate(new Pose2d(0,0,0));
        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(distance)
                .build();

        drive.followTrajectory(trajectoryForward);
    }

    public void driveBackward(double distance) {
        drive.setPoseEstimate(new Pose2d(0,0,0));
        Trajectory trajectoryBackward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(distance)
                .build();

        drive.followTrajectory(trajectoryBackward);
    }

    public void strafeRight(double distance) {
        drive.setPoseEstimate(new Pose2d(0,0,0));
        Trajectory trajectoryRight = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(distance)
                .build();

        drive.followTrajectory(trajectoryRight);
    }

    public void strafeLeft(double distance) {
        drive.setPoseEstimate(new Pose2d(0,0,0));
        Trajectory trajectoryLeft = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(distance)
                .build();

        drive.followTrajectory(trajectoryLeft);
    }

    public void lineTo(double x, double y) {
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        Trajectory line = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToConstantHeading(new Vector2d(x, y))
                .build();

        drive.followTrajectory(line);
    }

    public void turnWithDegrees(double degrees) {
        drive.setPoseEstimate(new Pose2d(0, 0,0));
        drive.turn(Math.toRadians(degrees));
    }

    public void liftArm(int liftHeight, double liftSpeed, double timeInSeconds) {
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(liftHeight);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        slideMotor.setPower(liftSpeed);
        while (slideMotor.isBusy() && opModeIsActive()) {
            slideMotor.setPower(liftSpeed);
        }
        slideMotor.setPower(0);
        sleep(500);
    }

    public void outtakeElapsed(double rotatePos, double timeInSeconds) {
        runtime.reset();
        while (runtime.seconds() < timeInSeconds) {
            outtakeServo.setPosition(rotatePos);
        }
    }

    public void clawElapsed(double rotatePos, double timeInSeconds) {
        runtime.reset();
        while (runtime.seconds() < timeInSeconds) {
            clawServo.setPosition(rotatePos);
        }
    }

    public void rotateElapsed(double rotatePos, double timeInSeconds) {
        runtime.reset();
        while (runtime.seconds() < timeInSeconds) {
            clawRotateServo.setPosition(rotatePos);
        }
    }

    public void resetArm(double liftSpeed, double timeInSeconds) {
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(liftSpeed);
        while (slideMotor.isBusy() && opModeIsActive() && !slideLimit.isPressed()) {
            slideMotor.setPower(liftSpeed);
        }
        slideMotor.setPower(0);
        sleep(500);
    }

    public void redInitOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );

        camera.setPipeline(redPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void blueInitOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );

        camera.setPipeline(bluePipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    class BlueObserverPipeline extends OpenCvPipeline {
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
                position = PropPosition.LEFT;
                Imgproc.rectangle(output, leftRect, foundColor, 2);
            } else if (rightAvgFin > centerAvgFin && rightAvgFin >= leftAvgFin) {
                position = PropPosition.RIGHT;
                Imgproc.rectangle(output, rightRect, foundColor, 2);
            } else {
                position = PropPosition.CENTER;
                Imgproc.rectangle(output, centerRect, foundColor, 2);
            }


            return output;
        }

        public PropPosition getPosition() {
            return position;
        }
    }

    class RedObserverPipeline extends OpenCvPipeline {
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

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


            Rect leftRect = new Rect(0, 0, 266, 448);
            Rect centerRect = new Rect(267, 0, 267, 448);
            Rect rightRect = new Rect(534, 0, 266, 448);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, centerRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            centerCrop = YCbCr.submat(centerRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(centerCrop, centerCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

            Scalar rightAvg = Core.mean(rightCrop);
            Scalar centerAvg = Core.mean(centerCrop);
            Scalar leftAvg = Core.mean(leftCrop);

            leftAvgFin = leftAvg.val[0];
            rightAvgFin = rightAvg.val[0];
            centerAvgFin = centerAvg.val[0];

            if (leftAvgFin > centerAvgFin && leftAvgFin > rightAvgFin) {
                position = PropPosition.LEFT;
                Imgproc.rectangle(output, leftRect, foundColor, 2);
            } else if (rightAvgFin > centerAvgFin && rightAvgFin >= leftAvgFin) {
                position = PropPosition.RIGHT;
                Imgproc.rectangle(output, rightRect, foundColor, 2);
            } else {
                position = PropPosition.CENTER;
                Imgproc.rectangle(output, centerRect, foundColor, 2);
            }


            return output;
        }

        public PropPosition getPosition() {
            return position;
        }
    }
}


