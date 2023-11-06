package org.firstinspires.ftc.teamcode.apriltag;

public class Quat {
    public final double a, b, c, d;

    public static final Quat ZERO = new Quat(0, 0, 0, 0);
    public static final Quat ONE = new Quat(1, Vec3.ZERO);

    public Quat(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public Quat(double a, Vec3 v) {
        this(a, v.x, v.y, v.z);
    }

    public Quat(double a) {
        this(a, Vec3.ZERO);
    }

    public Quat(Vec3 v) {
        this(0, v);
    }

    public double scalarPart() {
        return this.a;
    }

    public Vec3 vectorPart() {
        return new Vec3(this.b, this.c, this.d);
    }

    public double sqNorm() {
        return this.a * this.a + this.vectorPart().sqMagnitude();
    }

    public Quat conjugate() {
        return new Quat(
                this.scalarPart(),
                this.vectorPart().neg()
        );
    }

    public Quat inverse() {
        return this.conjugate().div(this.sqNorm());
    }

    public Quat add(Quat other) {
        return new Quat(
                this.a + other.a,
                this.b + other.b,
                this.c + other.c,
                this.d + other.d
        );
    }

    public Quat sub(Quat other) {
        return new Quat(
                this.a - other.a,
                this.b - other.b,
                this.c - other.c,
                this.d - other.d
        );
    }

    public Quat mul(Quat other) {
        return new Quat(
                (this.a * other.a) - (this.b * other.b) - (this.c * other.c) - (this.d * other.d),
                (this.a * other.b) + (this.b * other.a) + (this.c * other.d) - (this.d * other.c),
                (this.a * other.c) - (this.b * other.d) + (this.c * other.a) + (this.d * other.b),
                (this.a * other.d) + (this.b * other.c) - (this.c * other.b) + (this.d * other.a)
        );
    }

    public Quat mul(double other) {
        return new Quat(
                this.scalarPart() * other,
                this.vectorPart().mul(other)
        );
    }

    public Quat div(double other) {
        return new Quat(
                this.scalarPart() / other,
                this.vectorPart().div(other)
        );
    }

    public Quat conjugateBy(Quat other) {
        return other.mul(this).mul(other.inverse());
    }

    // Rotations

    public Quat compose(Quat other) {
        return other.mul(this);
    }

    public Vec3 apply(Vec3 point) {
        return new Quat(point).conjugateBy(this).vectorPart();
    }
}
