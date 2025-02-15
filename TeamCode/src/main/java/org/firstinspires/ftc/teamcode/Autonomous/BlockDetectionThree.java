package org.firstinspires.ftc.teamcode.Autonomous;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlockDetectionThree extends OpenCvPipeline {
    private Telemetry telemetry;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    public static final double objectWidthInRealWorldUnits = 3.75;
    public static final double focalLength = 728;

    public BlockDetectionThree(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Mat edges = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        // Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Define color ranges
        Scalar lowerYellow = new Scalar(11, 100, 20);
        Scalar upperYellow = new Scalar(35, 255, 255);
        Scalar lowerBlue = new Scalar(100, 100, 20);
        Scalar upperBlue = new Scalar(130, 255, 255);
        Scalar lowerRed0 = new Scalar(0, 100, 20);
        Scalar upperRed0 = new Scalar(10, 255, 255);
        Scalar lowerRed1 = new Scalar(170, 100, 20);
        Scalar upperRed1 = new Scalar(180, 255, 255);

        // Thresholding
        Mat maskRed = new Mat();
        Mat maskBlue = new Mat();
        Mat maskYellow = new Mat();
        Core.inRange(hsv, lowerRed0, upperRed0, maskRed);
        Mat tempRed = new Mat();
        Core.inRange(hsv, lowerRed1, upperRed1, tempRed);
        Core.bitwise_or(maskRed, tempRed, maskRed);
        Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

        // Apply Canny Edge Detection
        Imgproc.Canny(maskRed, edges, 100, 200);
        Imgproc.dilate(edges, edges, new Mat(), new Point(-1, -1), 2);

        // Find contours
        List<MatOfPoint> contoursRed = new ArrayList<>();
        Imgproc.findContours(edges, contoursRed, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Process detected contours
        for (MatOfPoint contour : contoursRed) {
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            if (rotatedRect.size.area() > 1000) {
                int numSubBlobs = (int) (rotatedRect.size.area() / 500);
                double newWidth = rotatedRect.size.width / Math.sqrt(numSubBlobs);
                double newHeight = rotatedRect.size.height / Math.sqrt(numSubBlobs);
                for (int i = 0; i < numSubBlobs; i++) {
                    double newX = rotatedRect.center.x + (i % Math.sqrt(numSubBlobs)) * newWidth;
                    double newY = rotatedRect.center.y + (i / Math.sqrt(numSubBlobs)) * newHeight;
                    Imgproc.rectangle(input, new Point(newX - newWidth / 2, newY - newHeight / 2),
                            new Point(newX + newWidth / 2, newY + newHeight / 2),
                            new Scalar(255, 0, 0), 2);
                }
            } else {
                Imgproc.drawContours(input, contoursRed, -1, new Scalar(0, 0, 255), 2);
                Point[] boxPoints = new Point[4];
                rotatedRect.points(boxPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(255, 0, 0), 2);
                }
                double distance = (3.5 * 728) / Math.max(rotatedRect.size.width, rotatedRect.size.height);
                double angle = rotatedRect.angle;
                Imgproc.putText(input, String.format("%.1f in, %.1f°, %.1f px", distance, angle, rotatedRect.size.width),
                        new Point(rotatedRect.center.x, rotatedRect.center.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);

            }


        }

        telemetry.addData("Red Contours", contoursRed.size());
        telemetry.update();

        return input;
    }
}
