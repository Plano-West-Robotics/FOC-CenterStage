package org.firstinspires.ftc.teamcode.freesight.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class APipeline extends OpenCvPipeline {
    ArrayList<double[]> frameList = new ArrayList<>();
    public double positionState;

    public Scalar lowHSV = new Scalar(0, 0, 0);
    public Scalar highHSV = new Scalar(0, 0, 0);
    public Scalar outline = new Scalar(0, 255, 0);

    private final Mat empty = new Mat();
    private final Mat main = new Mat();
    private final Mat masked = new Mat();
    private final Mat scaledThresh = new Mat();
    private final Mat scaledMask = new Mat();
    private final Mat threshold = new Mat();
    private final Mat hierarchy = new Mat();

    /**
     * @param input the frame to be manipulated
     * @return The altered matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        int width = input.width();
        int height = input.height();

        // Clear buffers
        empty.copyTo(main);
        empty.copyTo(masked);
        empty.copyTo(scaledMask);
        empty.copyTo(scaledThresh);
        empty.copyTo(threshold);
        empty.copyTo(hierarchy);
        //main = new Mat();

        Imgproc.cvtColor(input, main, Imgproc.COLOR_RGB2HSV);
        if (main.empty()) return input;

        /*
         * BLUE
         * Scalar lowHSV = new Scalar(55.3, 62.3, 53.8);
         * Scalar highHSV = new Scalar(213.9, 240.8, 255);
         */
        lowHSV = new Scalar(0.0, 18.4, 198.3);
        highHSV = new Scalar(255.0, 22.7, 226.7);
        //Mat threshold = new Mat();

        Core.inRange(main, lowHSV, highHSV, threshold);
        Imgproc.cvtColor(threshold, input, Imgproc.COLOR_GRAY2RGB);

        //masked = new Mat();


        Core.bitwise_and(main, main, masked, threshold);

        Scalar avg = Core.mean(masked, threshold);
        //scaledMask = new Mat();

        masked.convertTo(scaledMask, -1, 150 / avg.val[1], 0);

        double strictLowS = 62.3;
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0);
        Scalar strictHighHSV = new Scalar(255, 255, 255);
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        //contours, apply post processing to information
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        //Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);


        //threshRGB = new Mat();
        if (contours.size() > 0) {
            int index = 0;
            int area = 0;
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint bar = contours.get(i);

                if (Imgproc.boundingRect(bar).x < 2/5.) continue;

                int foo = bar.width() * bar.height();
                if (foo > area) {
                    index = i;
                    area = foo;
                }
            }

            MatOfPoint contour = contours.get(index);

            Rect boundingRect = Imgproc.boundingRect(contour);
            // center is ( x + w ) / 2

            // int point = (boundingRect.x + boundingRect.width) / 2;
            int point = boundingRect.x + boundingRect.width / 2;

            Imgproc.rectangle(
                    input,
                    boundingRect,
                    outline
            );

            positionState = ((double)point / width) - 3/5.;

            Imgproc.circle(
                    input,
                    new Point(
                            boundingRect.x + boundingRect.width / 2.0,
                            boundingRect.y + boundingRect.height / 2.0
                    ),
                    10,
                    outline
            );
            Imgproc.putText(
                    input,
                    positionState + "",
                    new Point(
                            boundingRect.x + boundingRect.width / 2.0,
                            boundingRect.y + boundingRect.height / 2.0
                    ),
                    Imgproc.FONT_ITALIC,
                    0.5,
                    new Scalar(255, 0, 255)
            );
        }

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }
        return input;

    }

    public void releaseFrames() {
        scaledThresh.release();
        scaledMask.release();
        main.release();
        masked.release();
        threshold.release();
        empty.release();
    }
}
