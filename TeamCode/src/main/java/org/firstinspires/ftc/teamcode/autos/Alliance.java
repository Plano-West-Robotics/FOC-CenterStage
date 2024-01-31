package org.firstinspires.ftc.teamcode.autos;

public enum Alliance {
    RED, BLUE;

    public boolean isRed() {
        switch (this) {
            case RED:
                return true;
            default:
                return false;
        }
    }

    public boolean isBlue() {
        switch (this) {
            case BLUE:
                return true;
            default:
                return false;
        }
    }
}
