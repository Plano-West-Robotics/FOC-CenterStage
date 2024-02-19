package org.firstinspires.ftc.teamcode.log;

public class Plank {
    private final Log log;
    private final String origin;

    Plank(Log log, String origin) {
        this.log = log;
        this.origin = origin;
    }

    public void addLine(String line) {
        this.log.addLine(this.origin, line);
    }

    public void addData(String key, Object value) {
        this.log.addData(this.origin, key, value);
    }
}
