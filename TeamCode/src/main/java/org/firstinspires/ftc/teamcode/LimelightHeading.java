package org.firstinspires.ftc.teamcode;

/**
 * Heading-frame conversion between Pedro Pathing and the FTC/Limelight field frame.
 *
 * Both frames are CCW-positive. They differ only by a +90 degree zero-reference offset:
 * Pedro 0 deg faces the red alliance goal; FTC/Limelight 0 deg faces the audience.
 * So FTC/Limelight heading = Pedro heading + 90, modulo 360.
 */
public final class LimelightHeading {
    private LimelightHeading() {}

    public static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    public static double pedroHeadingToLimelightDeg(double pedroHeadingRad) {
        double deg = Math.toDegrees(pedroHeadingRad) + 90.0;
        // Normalize to -180 to +180, matching FTC IMU convention
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    public static double limelightDegToPedroHeadingRad(double limelightDeg) {
        double deg = limelightDeg - 90.0;
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return Math.toRadians(deg);
    }
}
