package org.firstinspires.ftc.teamcode.computerVision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat rightCrop;
    Mat leftCrop;
    Mat centerCrop;
    double leftAvgFin;
    double rightAvgFin;
    double centerAvgFin;
    Mat output = new Mat();    
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    //    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


    public enum Spike {
        LEFT,
        RIGHT,
        CENTER
    }

    private volatile Spike position;

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
            position = Spike.LEFT;
//            telemetry.addLine("Left");
        }
        else if (centerAvgFin > rightAvgFin && centerAvgFin > leftAvgFin) {
            position = Spike.CENTER;
//            telemetry.addLine("Center");
        }
        else {
            position = Spike.RIGHT;
//            telemetry.addLine("Right");
        }


        return output;
    }
    public Spike getSpikePosition(){
        return position;
    }
}
