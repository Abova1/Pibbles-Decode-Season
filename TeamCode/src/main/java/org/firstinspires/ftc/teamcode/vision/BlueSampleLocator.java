package org.firstinspires.ftc.teamcode.vision;

import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueSampleLocator extends OpenCvPipeline {

    Detection sample = new Detection();

    public int yellowCenterY = Values.yellowCenterY, yellowCenterX = Values.yellowCenterX;

    public int yellowOrientation = Values.yellowOrientation;

    public int yellowHeight = Values.yellowHeight, yellowWidth = Values.yellowWidth;

    public int blueCenterX = Values.blueCenterX, blueCenterY = Values.blueCenterY;
    public int blueOrientation = Values.blueOrientation;
    public int blueHeight = Values.blueHeight, blueWidth = Values.blueWidth;
    public boolean SS = Values.SS;
    public String YellowStreamInfo = Values.YellowStreamInfo;
    public String blueStreamInfo = Values.BlueStreamInfo;
    public String YellowSSInfo = Values.YellowSSInfo;
    public String blueSSInfo = Values.BlueSSInfo;

    Mat hsv = new Mat();
    Mat yellowMask = new Mat();
    Mat blueMask = new Mat();
    double CameraMidPointX;
    double CameraMidPointY;

    @Override
    public Mat processFrame(Mat input) {

        CameraMidPointX = (double) input.width() / 2;
        CameraMidPointY = (double) input.height() / 2;

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerYellow = new Scalar(20, 65, 75);
        Scalar upperYellow = new Scalar(65, 255, 255);

        Scalar lowerBlue = new Scalar(100, 150, 100);
        Scalar upperBlue = new Scalar(165, 255, 255);

        Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

        Core.inRange(hsv, lowerBlue, upperBlue, blueMask);


        List<MatOfPoint> yellowContours = new ArrayList<>();
        List<MatOfPoint> blueContours = new ArrayList<>();

        Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(blueMask, blueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.circle(input, new Point( CameraMidPointX , CameraMidPointY), 6, new Scalar(20, 255, 50), -1);

        sample.detection(input,
                "Yellow", yellowMask, yellowContours,
                yellowWidth, yellowHeight,
                yellowCenterX, yellowCenterY, yellowOrientation,
                0, 0
        );


        sample.detection(input,
                "blue", blueMask, blueContours,
                blueWidth, blueHeight,
                blueCenterX, blueCenterY, blueOrientation,
                0, 0
        );

        YellowStreamInfo = sample.YellowInfo;
        blueStreamInfo = sample.BlueInfo;

        if(SS){
            String Path =  Environment.getExternalStorageDirectory().getPath() + System.currentTimeMillis() + ".png";

            Imgcodecs.imwrite(Path, input);

            YellowSSInfo = YellowStreamInfo;
            blueSSInfo = blueStreamInfo;


            SS = false;
        }

        return input;

    }
}