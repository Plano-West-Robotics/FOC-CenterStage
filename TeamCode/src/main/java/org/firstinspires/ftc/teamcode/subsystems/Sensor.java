package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.opencv.core.Scalar;

public class Sensor {
    Hardware hw;
    public State top_state;
    public State bottom_state;

    /**
     * Which pixel is detected
     */
    public enum State {
        GREEN, PURPLE, YELLOW, WHITE, NONE;

        public RevBlinkinLedDriver.BlinkinPattern toColorPattern() {
            // enum hell
            switch (this) {
                case GREEN:
                    return RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
                case PURPLE:
                    return RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                case YELLOW:
                    return RevBlinkinLedDriver.BlinkinPattern.GOLD;
                case WHITE:
                    return RevBlinkinLedDriver.BlinkinPattern.GRAY;
                case NONE:
                default:
                    return RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
        }
    }

    public Sensor(Hardware hw) {
        this.hw = hw;
        top_state = State.NONE;
        bottom_state = State.NONE;
    }

    public void update() {

        ColorScalar top = new ColorScalar(
                this.hw.top.getNormalizedColors().red,
                this.hw.top.getNormalizedColors().green,
                this.hw.top.getNormalizedColors().blue
        );
        ColorScalar bottom = new ColorScalar(
                this.hw.bottom.getNormalizedColors().red,
                this.hw.bottom.getNormalizedColors().green,
                this.hw.bottom.getNormalizedColors().blue
        );
        top.cvtRGBToHSV();
        bottom.cvtRGBToHSV();
        double top_distance = this.hw.top.getDistance(DistanceUnit.MM);
        double bottom_distance = this.hw.bottom.getDistance(DistanceUnit.MM);
        top_state = getMost(top, top_distance);
        bottom_state = getMost(bottom, bottom_distance);

    }

    /**
     * Use HSV color wheel threshold to determine the pixel colour
     * @param scr HSV colour scalar to check
     * @return Color of Pixel
     */
    public static State getMost(ColorScalar scr, double dist) {
        double h = scr.val[0];
        double s = scr.val[1];

        if (dist > 20) {
            return State.NONE;
        }

        
        if (h >= 95 && h <= 145) {
            return State.GREEN;
        } else if (h >= 200 && h <= 290) {
            return State.PURPLE;
        } else if (h >= 40 && h <= 90) {
            return State.YELLOW;
        } else if (s <= 50) {
            return State.WHITE;
        }

        return State.NONE;
    }

    /**
     * Specific use Scalar for RGB/HSV colour values
     */
    public static class ColorScalar extends Scalar {
        boolean isHSV = false;

        public ColorScalar(double v0, double v1, double v2) {
            super(v0, v1, v2);
        }

        /**
         * Converts the RGB values to HSV values
         */
        public void cvtRGBToHSV() {
            if (this.isHSV) return;

            double r = this.val[0];
            double g = this.val[1];
            double b = this.val[2];

            // h, s, v = hue, saturation, value
            double cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
            double cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
            double diff = cmax - cmin; // diff of cmax and cmin.
            double h = -1, s = -1;

            // if cmax and cmax are equal then h = 0
            if (cmax == cmin)
                h = 0;
            // if cmax equal r then compute h
            else if (cmax == r)
                h = (60 * ((g - b) / diff) + 360) % 360;
            // if cmax equal g then compute h
            else if (cmax == g)
                h = (60 * ((b - r) / diff) + 120) % 360;
            // if cmax equal b then compute h
            else if (cmax == b)
                h = (60 * ((r - g) / diff) + 240) % 360;

            // if cmax equal zero
            if (cmax == 0)
                s = 0;
            else
                s = (diff / cmax) * 100;

            // compute v
            double v = cmax * 100;
            this.set(new double[]{h, s, v});
            this.isHSV = true;
        }
    }


}
