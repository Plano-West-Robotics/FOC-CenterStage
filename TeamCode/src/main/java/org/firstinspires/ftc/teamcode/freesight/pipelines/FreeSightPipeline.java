package org.firstinspires.ftc.teamcode.freesight.pipelines;

//import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


//@Config
public class FreeSightPipeline extends OpenCvPipeline {

    public enum Prop {
        PURPLE, ORANGE, NONE
    }

    public enum Side {
        LEFT, RIGHT, MIDDLE
    }

    public Side positionState;

    public Scalar lowHSV = new Scalar(0, 0, 0);
    public Scalar highHSV = new Scalar(0, 0, 0);
    public Scalar boxOutline = new Scalar(0, 255, 0);
    public Scalar textColor = new Scalar(255, 0, 255);
    public Prop colorState = Prop.NONE;

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

        // Convert main to HSV
        Imgproc.cvtColor(input, main, Imgproc.COLOR_RGB2HSV);
        if (main.empty()) return input;

        if (colorState == Prop.PURPLE) {
            lowHSV = new Scalar(106.3, 66.3, 60.8);
            highHSV = new Scalar(151.9, 178.8, 219.0);
        } else if (colorState == Prop.ORANGE) {
            lowHSV = new Scalar(0, 162.9, 107.7);
            highHSV = new Scalar(15.6, 255, 184.2);
        }

        // find colors that are within the HSV bounds
        Core.inRange(main, lowHSV, highHSV, threshold);
        Imgproc.cvtColor(threshold, input, Imgproc.COLOR_GRAY2RGB);



        Core.bitwise_and(main, main, masked, threshold);

        Scalar avg = Core.mean(masked, threshold);

        masked.convertTo(scaledMask, -1, 150 / avg.val[1], 0);

        // todo: revert this commit iff it breaks

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(scaledMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);


        if (contours.size() > 0) {
            int index = 0;
            double maxArea = 0;
            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxArea) {
                    index = i;
                    maxArea = area;
                }
            }

            MatOfPoint maxContour = contours.get(index);

            Rect boundingRect = Imgproc.boundingRect(maxContour);

            int centerPoint = boundingRect.x + boundingRect.width / 2;

            // draw rectangle around the largest contour
            Imgproc.rectangle(
                    input,
                    boundingRect,
                    boxOutline
            );

            // find which third of the screen the center point is in
            // that third corresponds to the spike mark that the team prop is on
            int bigX;
            if (centerPoint < width / 3) {
                positionState = Side.LEFT;
                bigX = 0;
            } else if (centerPoint > width / 1.5) {
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
                    boxOutline
            );
            Imgproc.circle(
                    input,
                    new Point(
                            boundingRect.x + boundingRect.width / 2.0,
                            boundingRect.y + boundingRect.height / 2.0
                    ),
                    10,
                    boxOutline
            );
            Imgproc.putText(
                    input,
                    positionState.toString(),
                    new Point(
                            boundingRect.x + boundingRect.width / 2.0,
                            boundingRect.y + boundingRect.height / 2.0
                    ),
                    Imgproc.FONT_ITALIC,
                    0.5,
                    textColor
            );
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
