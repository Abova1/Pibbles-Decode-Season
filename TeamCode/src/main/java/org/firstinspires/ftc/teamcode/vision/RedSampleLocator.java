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

public class RedSampleLocator extends OpenCvPipeline{

    Detection sample = new Detection();

    public int yellowCenterY = Values.yellowCenterY, yellowCenterX = Values.yellowCenterX;

    public int yellowOrientation = Values.yellowOrientation;

    public int yellowHeight = Values.yellowHeight, yellowWidth = Values.yellowWidth;

    public int redCenterX = Values.redCenterX, redCenterY = Values.redCenterY;
    public int redOrientation = Values.redOrientation;
    public int redHeight = Values.redHeight, redWidth = Values.redWidth;

    public boolean SS = Values.SS;
    public String YellowStreamInfo = Values.YellowStreamInfo;
    public String redStreamInfo = Values.redStreamInfo;
    public String YellowSSInfo = Values.YellowSSInfo;
    public String redSSInfo = Values.redSSInfo;

    Mat hsv = new Mat();
    Mat yellowMask = new Mat();
    Mat redMask = new Mat();
    double CameraMidPointX;
    double CameraMidPointY;

    @Override
    public Mat processFrame(Mat input) {

        CameraMidPointX = (double) input.width() / 2;
        CameraMidPointY = (double) input.height() / 2;

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerYellow = new Scalar(20, 65, 75);
        Scalar upperYellow = new Scalar(65, 255, 255);

        Scalar lowerRed = new Scalar(170, 100, 150);
        Scalar upperRed = new Scalar(180, 255, 255);

        Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

        Core.inRange(hsv, lowerRed, upperRed, redMask);



        List<MatOfPoint> yellowContours = new ArrayList<>();
        List<MatOfPoint> redContours = new ArrayList<>();

        Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(redMask, redContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.circle(input, new Point( CameraMidPointX , CameraMidPointY), 6, new Scalar(20, 255, 50), -1);

        sample.detection(input,
                "Yellow", yellowMask, yellowContours,
                yellowWidth, yellowHeight,
                yellowCenterX, yellowCenterY, yellowOrientation,
                0, 0
        );


        sample.detection(input,
                    "Red", redMask, redContours,
                    redWidth, redHeight,
                    redCenterX, redCenterY, redOrientation,
                    0, 0
        );

        YellowStreamInfo = sample.YellowInfo;
        redStreamInfo = sample.RedInfo;

        if(SS){
                String Path =  Environment.getExternalStorageDirectory().getPath() + System.currentTimeMillis() + ".png";

                Imgcodecs.imwrite(Path, input);

                YellowSSInfo = YellowStreamInfo;
                redSSInfo = redStreamInfo;


                SS = false;
        }

        return input;

    }
}