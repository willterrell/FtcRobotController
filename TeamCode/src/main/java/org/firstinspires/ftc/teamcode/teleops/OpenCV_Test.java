package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;

import java.util.ArrayList;
import java.util.List;

public class OpenCV_Test extends OpenCvPipeline{
    Telemetry t;
    Mat mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();

    //Yellow
    Scalar lowHSV = new Scalar(20, 100, 100);
    Scalar highHSV = new Scalar(30, 255, 255);

    // morphology
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    OpenCV_Test(Telemetry t) { this.t = t;}


    @Override
    public Mat processFrame(Mat input)
    {
        //This turns the image to HSV the thersholds the image
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);
        Core.inRange(mat, lowHSV, highHSV, mat);
        //morphMask(mat, morphedThreshold);

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            //Rect boundRect = Imgproc.boundingRect(contours);
           // double centerX = boundRect.x + (boundRect.width / 2);
            //double centerY = boundRect.y + (boundRect.height / 2);

            Imgproc.drawContours(mat, contours, i, new Scalar(255, 255, 255), -1);
        }
        return mat;
    }

    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

}/**
    Telemetry t;
    Mat mat = new Mat();

    static final Rect Left_ROI = new Rect(
        new Point(60, 35),
        new Point(200, 75));


    public OpenCV_Test(Telemetry t)
    {
        this.t = t;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        /**
        color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
        'white': [[180, 18, 255], [0, 0, 231]],
        'red1': [[180, 255, 255], [159, 50, 70]],
        'red2': [[9, 255, 255], [0, 50, 70]],
        'green': [[89, 255, 255], [36, 50, 70]],
        'blue': [[128, 255, 255], [90, 50, 70]],
        'yellow': [[35, 255, 255], [25, 50, 70]],
        'purple': [[158, 255, 255], [129, 50, 70]],
        'orange': [[24, 255, 255], [10, 50, 70]],
        'gray': [[180, 18, 230], [0, 0, 40]]}

        //Scalar lowHSV = new Scalar(90, 50, 70);
        //Scalar highHSV = new Scalar(128,255,255);
        Scalar lowHSV = new Scalar(20, 100, 100);
        Scalar highHSV = new Scalar(30, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat center = mat.submat(Left_ROI);

        double leftValue = Core.sumElems(center).val[0] / Left_ROI.area() / 255;

        center.release();

        t.addData("center raw value:", (int)Core.sumElems(center).val[0]);
        t.addData("center percentage:", Math.round(leftValue * 100) + "%");

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        t.update();
        return mat;
    }
    **/


/**
class SamplePipeline extends OpenCvPipeline{
    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    int avg;

    void inputToY(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0)
    }

    public void init(Mat firstFrame)
    {
        inputToY(firstFrame);
    }

    public Mat processFrame(Mat input)
    {
        inputToY(input);
        System.out.println("processing requested");
        avg = (int) Core.mean(Y).val[0];
        YCrCb.release();
        Y.release();
        return input;
    }

    public int getAnalysis()
    {
        return avg;
    }
}
**/