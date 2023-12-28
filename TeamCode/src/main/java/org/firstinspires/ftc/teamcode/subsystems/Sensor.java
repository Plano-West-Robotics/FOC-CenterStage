package org.firstinspires.ftc.teamcode.subsystems;

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

        public LED.ColorPattern toColorPattern() {
            // enum hell
            switch (this) {
                case GREEN:
                    return LED.ColorPattern.GREEN;
                case PURPLE:
                    return LED.ColorPattern.PURPLE;
                case YELLOW:
                    return LED.ColorPattern.YELLOW;
                case WHITE:
                    return LED.ColorPattern.WHITE;
                case NONE:
                default:
                    return LED.ColorPattern.NONE;
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
                this.hw.top.red(),
                this.hw.top.blue(),
                this.hw.top.green()
        );
        ColorScalar bottom = new ColorScalar(
                this.hw.bottom.red(),
                this.hw.bottom.blue(),
                this.hw.bottom.green()
        );
        top.cvtRGBToHSV();
        bottom.cvtRGBToHSV();
        top_state = getMost(top);
        bottom_state = getMost(bottom);
    }

    /**
     * Use HSV color wheel threshold to determine the pixel color
     * @param scr HSV colour scalar to check
     * @return Color of Pixel
     */
    public static State getMost(ColorScalar scr) {
        double h = scr.val[0];
        double s = scr.val[1];
        if (s <= 10.0) {
            return State.WHITE;
        } else if (h >= 85 && h <= 145) {
            return State.GREEN;
        } else if (h >= 263 && h <= 290) {
            return State.PURPLE;
        } else if (h >= 40 && h <= 60) {
            return State.YELLOW;
        }
        return State.NONE;
    }

    /**
     * Specific use Scalar for RGB/HSV color values
     */
    static class ColorScalar extends Scalar {
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
            //COPIED FROM https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/
            r = r / 255.0;
            g = g / 255.0;
            b = b / 255.0;

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
