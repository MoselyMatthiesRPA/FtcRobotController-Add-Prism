package org.firstinspires.ftc.teamcode.lawrence.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class RobotConstants {
    private RobotConstants() {}

    // ── Flywheel ──────────────────────────────────────────────────────────────
    public static final double MIN_RPM = 0.0;
    public static final double MAX_RPM = 5800.0;
    public static double RPM_AT_1M  = 2125;
    public static double RPM_AT_2M  = 2500;
    public static double RPM_AT_FAR = 3000;

    // ── Hood tilt (servo positions 0.0–1.0) ──────────────────────────────────
    public static final double MIN_TILT = 0.02;
    public static double MAX_TILT    = 0.65;
    public static double TILT_AT_1M  = 0.25;
    public static double TILT_AT_2M  = 0.60;
    public static double TILT_AT_FAR = 0.70;

    // ── Intake RPM ────────────────────────────────────────────────────────────
    public static double INTAKING_RPM  = 900;
    public static double SHOOTING_RPM  = 600;
    public static double FAR_RPM       = 250;
    public static double CLOSE_RPM     = 500;

    // ── Stopper servo positions ───────────────────────────────────────────────
    public static final double STOPPER_UP   = 0.0;
    public static final double STOPPER_DOWN = 0.13;

    // ── Distance calibration ──────────────────────────────────────────────────
    public static final double INCHES_PER_METER = 39.3701;
    public static final double RAW_AT_1M_IN = 40.0;
    public static final double RAW_AT_2M_IN = 76.5;
    public static final double TRUE_1M_IN   = INCHES_PER_METER;
    public static final double TRUE_2M_IN   = 2.0 * INCHES_PER_METER;

    // Linear correction: corrected = DIST_A*raw + DIST_B
    // Fit to (RAW_AT_1M → TRUE_1M) and (RAW_AT_2M → TRUE_2M)
    public static final double DIST_A =
            (TRUE_2M_IN - TRUE_1M_IN) / (RAW_AT_2M_IN - RAW_AT_1M_IN);
    public static final double DIST_B = TRUE_1M_IN - (DIST_A * RAW_AT_1M_IN);

    // Distance threshold: beyond this use FAR constants instead of the linear model
    public static final double FAR_THRESHOLD_IN = 105.0;

    // ── Turret ────────────────────────────────────────────────────────────────
    // Small intentional offset so the turret leads the goal center slightly
    public static double TURRET_TARGET_OFFSET = 2.0;

    // ── Vision (Limelight pose fusion) ───────────────────────────────────────
    // Reject frames older than this
    public static double MAX_LL_STALENESS_MS  = 100.0;
    // Reject tags with ambiguity above this (unused in MT2 but kept for reference)
    public static double MAX_TAG_AMBIGUITY    = 0.7;
    // Reject tags farther than this from the camera
    public static double MAX_TAG_DISTANCE_M   = 4.0;
    // Reject pose updates smaller than this (prevents jitter while stationary)
    public static double MIN_POSE_JUMP_IN     = 0.5;
    // Reject pose updates larger than this (wild outliers)
    public static double MAX_POSE_JUMP_IN     = 24.0;
}
