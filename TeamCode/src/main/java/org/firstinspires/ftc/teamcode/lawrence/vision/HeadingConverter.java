package org.firstinspires.ftc.teamcode.lawrence.vision;

/**
 * Converts headings between Pedro Pathing and the FTC/Limelight field frame.
 *
 * Both frames are CCW-positive and differ only by a +90° zero-reference offset:
 *   Pedro  0° → faces the red alliance goal
 *   FTC/LL 0° → faces the audience
 *
 * Conversion: FTC/LL heading = Pedro heading + 90°  (mod 360)
 */
public final class HeadingConverter {
    private HeadingConverter() {}

    public static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    public static double pedroToLimelightDeg(double pedroHeadingRad) {
        return normalizeDeg(Math.toDegrees(pedroHeadingRad) + 90.0);
    }

    public static double limelightToPedroRad(double limelightDeg) {
        return Math.toRadians(normalizeDeg(limelightDeg - 90.0));
    }
}
