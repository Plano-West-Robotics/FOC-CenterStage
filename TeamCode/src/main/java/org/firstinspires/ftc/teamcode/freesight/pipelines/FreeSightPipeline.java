package org.firstinspires.ftc.teamcode.freesight.pipelines;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


@Config
public class FreeSightPipeline extends OpenCvPipeline {

    public enum Prop {
        BLUE, ORANGE, NONE
    }

    public enum Side {
        LEFT, RIGHT, MIDDLE
    }

    ArrayList<double[]> frameList = new ArrayList<>();
    public Side positionState;

    public Scalar lowHSV = new Scalar(0, 0, 0);
    public Scalar highHSV = new Scalar(0, 0, 0);
    public Scalar outline = new Scalar(0, 255, 0);
    public Prop colorState = Prop.NONE;

    private Mat empty = new Mat();
    private Mat main = new Mat();
    private Mat masked = new Mat();
    private Mat scaledThresh = new Mat();
    private Mat scaledMask = new Mat();
    private Mat threshold = new Mat();
    private Mat hierarchy = new Mat();


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
        if (colorState == Prop.BLUE) {
            lowHSV = new Scalar(55.3, 62.3, 53.8);
            highHSV = new Scalar(213.9, 240.8, 255);
        } else if (colorState == Prop.ORANGE) {
            lowHSV = new Scalar(0, 106.3, 198.3);
            highHSV = new Scalar(14.2, 255, 255);
        }
        //Mat threshold = new Mat();

        Core.inRange(main, lowHSV, highHSV, threshold);

        //masked = new Mat();


        Core.bitwise_and(main, main, masked, threshold);

        Scalar avg = Core.mean(masked, threshold);
        //scaledMask = new Mat();

        masked.convertTo(scaledMask, -1, 150 / avg.val[1], 0);

        //scaledThresh = new Mat();
        double strictLowS;
        //you probably want to tune this
        if (colorState == Prop.BLUE)
            strictLowS = 62.3;
        else
            strictLowS = 86.4; // orange
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, 255, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
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

            int bigX;
            if (point < width / 3) {
                positionState = Side.LEFT;
                bigX = 0;
            } else if (point > width / 1.5) {
                positionState = Side.RIGHT;
                bigX = width * 2 / 3;
            } else {
                positionState = Side.MIDDLE;
                bigX = width / 3;
            }

            Imgproc.rectangle(
                    input,
                    new Rect(bigX,
                            0,
                            width / 3,
                            height
                    ),
                    outline
            );
            Imgproc.circle(
                    input,
                    new Point(
                            boundingRect.x + boundingRect.width / 2,
                            boundingRect.y + boundingRect.height / 2
                    ),
                    10,
                    outline
            );
            Imgproc.putText(
                    input,
                    positionState.toString(),
                    new Point(
                            boundingRect.x + boundingRect.width / 2,
                            boundingRect.y + boundingRect.height / 2
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
