package org.firstinspires.ftc.teamcode.apriltag;

public class Vec3 {
    public final double x, y, z;

    public static final Vec3 ZERO = new Vec3(0, 0, 0);

    // TODO: units. im too lazy to do that rn
    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double sqMagnitude() {
        return this.dot(this);
    }

    public Vec3 neg() {
        return new Vec3(-this.x, -this.y, -this.z);
    }

    public Vec3 add(Vec3 other) {
        return new Vec3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    public Vec3 sub(Vec3 other) {
        return new Vec3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    public Vec3 mul(double other) {
        return new Vec3(this.x * other, this.y * other, this.z * other);
    }

    public Vec3 div(double other) {
        return new Vec3(this.x / other, this.y / other, this.z / other);
    }

    public double dot(Vec3 other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    public Vec3 cross(Vec3 other) {
        return new Vec3(
                this.y * other.z - this.z * other.y,
                this.z * other.x - this.x * other.z,
                this.x * other.y - this.y * other.x
        );
    }
}
