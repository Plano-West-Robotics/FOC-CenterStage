package org.firstinspires.ftc.teamcode.apriltag;

import org.opencv.core.Point;

public class TagGroup {
    public static class Tag {
        public int id;
        public Point pos;
        public double size;

        public Tag(int id, Point pos, double size) {
            this.id = id;
            this.pos = pos;
            this.size = size;
        }
    }

    public Pose pose;
    public Tag[] tags;

    public TagGroup(Pose pose, Tag... tags) {
        this.pose = pose;
        this.tags = tags;
    }
}
