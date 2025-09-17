package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

class Detection {

    Mat roi;
    public String YellowInfo;
    public String BlueInfo;
    public String RedInfo;
    //Just add the color info we need

    public void detection(
            Mat input,
            String Color,
            Mat Mask,
            List<MatOfPoint> Contours,
            int width, int height,
            int CenterX, int CenterY,
            int Orientation,
            int labelX, int labelY) {

        if (!Contours.isEmpty()) {
            double largestArea = 0;
            Rect largestRectangle = null;

            for (MatOfPoint contour : Contours) {
                Rect rectangle = Imgproc.boundingRect(contour);

                width = rectangle.width;

                height = rectangle.height;

                if (width > 55 && width < 160 && height > 55 && height < 160) {

                    roi = new Mat(Mask, rectangle);

                    int Pixels = Core.countNonZero(roi);

                    int totalPixels = width * height;

                    double Percentage = (double) Pixels / totalPixels * 100.0;

                    int difference = Math.abs(width - height);

                    boolean square = difference < 28.5;

                    boolean tooMuch = Percentage > 63;

                    if (square && tooMuch) {
                        continue;
                    }

                    if (rectangle.area() > largestArea) {
                        largestArea = rectangle.area();
                        largestRectangle = rectangle;
                    }
                }
            }

            if (largestRectangle != null) {
                CenterX = largestRectangle.x + largestRectangle.width / 2;
                CenterY = largestRectangle.y + largestRectangle.height / 2;

                height = largestRectangle.height;
                width = largestRectangle.width;

                if(largestRectangle.height > largestRectangle.width){
                    Orientation = 0;
                    Imgproc.putText(
                            input,
                            Color + ": Vertical",
                            new Point(labelX, labelY),
                            Imgproc.FONT_HERSHEY_DUPLEX,
                            0.85,
                            new Scalar(255, 255, 0))
                    ;
                }
                else if (largestRectangle.width > largestRectangle.height){
                    Orientation = 180;
                    Imgproc.putText(
                            input,
                            Color + ": Horizontal",
                            new Point(labelX, labelY),
                            Imgproc.FONT_HERSHEY_DUPLEX,
                            0.85,
                            new Scalar(255, 255, 0))
                    ;
                }


                Imgproc.rectangle(input, largestRectangle, new Scalar(100, 0, 255), 2);
                Imgproc.circle(input, new Point(CenterX, CenterY), 4, new Scalar(255, 0, 0), -1);
            }
        } else {
            CenterX = -1;
            CenterY = -1;
        }

        String SSinfo1 = Color + " height: " + height + " | "
                + Color + " Width: " + width + " | "
                + Color + " X: " + CenterX + " | "
                + Color + " Y: " + CenterY + " | "
                + Color + " Orientation: " + Orientation
        ;

        if (Color.equals("Blue")) {
           BlueInfo = SSinfo1;
        } else if (Color.equals("Yellow")) {
           YellowInfo = SSinfo1;
        } else if (Color.equals("Red")) {
            RedInfo = SSinfo1;
        }


    }



}