package org.firstinspires.ftc.teamcode.Autonomous;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlockDetectionPipeline extends OpenCvPipeline {
    private Telemetry telemetry;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 349*2;  // Replace with the focal length of the camera in pixels

    public BlockDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

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

        Imgproc.erode(maskRed, maskRed, new Mat(), new Point(-1, -1), 2);
        Imgproc.erode(maskBlue, maskBlue, new Mat(), new Point(-1, -1), 2);
        Imgproc.erode(maskYellow, maskYellow, new Mat(), new Point(-1, -1), 2);

        List<MatOfPoint> contoursRed = new ArrayList<>();
        List<MatOfPoint> contoursBlue = new ArrayList<>();
        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(maskRed, contoursRed, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskBlue, contoursBlue, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskYellow, contoursYellow, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contoursRed) {
            Imgproc.drawContours(input, contoursRed, -1, new Scalar(0, 0, 255), 2);
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            Point[] boxPoints = new Point[4];
            rotatedRect.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(255, 0, 0), 2);
            }

            double width = rotatedRect.size.width;
            double height = rotatedRect.size.height;
            double angle = rotatedRect.angle;


            double distance = (3.75 * focalLength) / Math.max(width, height);

            Imgproc.putText(input, String.format("%.1f in, %.1f°, %.1f px", distance, angle, width),
                    new Point(rotatedRect.center.x, rotatedRect.center.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);
        }
        for (MatOfPoint contour : contoursBlue) {
            Imgproc.drawContours(input, contoursBlue, -1, new Scalar(255, 0, 0), 2);
        }
        for (MatOfPoint contour : contoursYellow) {
            Imgproc.drawContours(input, contoursYellow, -1, new Scalar(0, 255, 255), 2);
        }

        telemetry.addData("Red Contours", contoursRed.size());
        telemetry.addData("Blue Contours", contoursBlue.size());
        telemetry.addData("Yellow Contours", contoursYellow.size());
        telemetry.update();

        return input;
    }






}
