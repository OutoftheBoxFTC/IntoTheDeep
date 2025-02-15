package org.firstinspires.ftc.teamcode.Autonomous;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlockDetectionTwo extends OpenCvPipeline {
    private Telemetry telemetry;
    public static final double OBJECT_WIDTH_REAL = 3.75;
    public static final double FOCAL_LENGTH = 728;
    private static final double MAX_BLOCK_AREA = 5000;

    public BlockDetectionTwo(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerYellow = new Scalar(11, 100, 20);
        Scalar upperYellow = new Scalar(35, 255, 255);
        Scalar lowerBlue = new Scalar(100, 100, 20);
        Scalar upperBlue = new Scalar(130, 255, 255);
        Scalar lowerRed0 = new Scalar(0, 100, 20);
        Scalar upperRed0 = new Scalar(10, 255, 255);
        Scalar lowerRed1 = new Scalar(170, 100, 20);
        Scalar upperRed1 = new Scalar(180, 255, 255);

        Mat maskRed = new Mat();
        Mat maskBlue = new Mat();
        Mat maskYellow = new Mat();

        Core.inRange(hsv, lowerRed0, upperRed0, maskRed);
        Mat tempRed = new Mat();
        Core.inRange(hsv, lowerRed1, upperRed1, tempRed);
        Core.bitwise_or(maskRed, tempRed, maskRed);


        Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

        separateAndProcessContours(maskRed, input, "Red", new Scalar(0, 0, 255));
        separateAndProcessContours(maskBlue, input, "Blue", new Scalar(255, 0, 0));
        separateAndProcessContours(maskYellow, input, "Yellow", new Scalar(0, 255, 255));

        telemetry.update();
        return input;
    }

    private void separateAndProcessContours(Mat mask, Mat input, String color, Scalar drawColor) {
        Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7)));

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 500) continue;

            if (area > MAX_BLOCK_AREA) {
                splitContour(contour, input, drawColor);
                continue;
            }

            processSingleContour(contour, input, color, drawColor);
        }
    }

    private void splitContour(MatOfPoint contour, Mat input, Scalar drawColor) {
        Rect boundingBox = Imgproc.boundingRect(contour);
        int newWidth = boundingBox.width / 2;
        int centerX = boundingBox.x + boundingBox.width / 2;

        Rect leftRect = new Rect(boundingBox.x, boundingBox.y, newWidth, boundingBox.height);
        Rect rightRect = new Rect(centerX, boundingBox.y, newWidth, boundingBox.height);

        Imgproc.rectangle(input, leftRect.tl(), leftRect.br(), drawColor, 2);
        Imgproc.rectangle(input, rightRect.tl(), rightRect.br(), drawColor, 2);
    }

    private void processSingleContour(MatOfPoint contour, Mat input, String color, Scalar drawColor) {
        RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
        Point[] boxPoints = new Point[4];
        rotatedRect.points(boxPoints);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], drawColor, 2);
        }



        double width = rotatedRect.size.width;
        double height = rotatedRect.size.height;
        double angle = rotatedRect.angle;
        if (width < height) {
            angle += 90;
        }
        double distance = (OBJECT_WIDTH_REAL * FOCAL_LENGTH) / Math.max(width, height);

        Imgproc.putText(input, String.format("%.1f in, %.1f°", distance, angle),
                new Point(rotatedRect.center.x, rotatedRect.center.y - 10),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 2);

        telemetry.addData(color + " Block Distance", String.format("%.1f inches", distance));
        telemetry.addData(color + " Block Angle", String.format("%.1f degrees", angle));


    }

    public double getAngleOfChosenBlock()
    {
        return 0;
    }

    public double getDistanceOfChosenBlock()
    {
        return 0;
    }

    public void chooseBlock()
    {
        return;
    }

    public void updateVision()
    {
        return;
    }

}
