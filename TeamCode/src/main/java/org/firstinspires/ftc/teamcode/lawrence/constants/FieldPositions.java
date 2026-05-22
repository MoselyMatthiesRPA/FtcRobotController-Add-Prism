package org.firstinspires.ftc.teamcode.lawrence.constants;

public final class FieldPositions {
    private FieldPositions() {}

    // Goal positions in inches, Pedro coordinate system.
    // Derived from FTC AprilTag positions via: pedroX = ftcY + 72, pedroY = -ftcX + 72
    // FTC Red Goal tag: (-58.3727, 55.6425) → Pedro (127.6425, 130.3727)
    // FTC Blue Goal tag: (58.3727, 55.6425) → Pedro (127.6425, 13.6273)
    public static final double RED_GOAL_X  = 127.6425;
    public static final double RED_GOAL_Y  = 130.3727;
    public static final double BLUE_GOAL_X = 127.6425;
    public static final double BLUE_GOAL_Y =  13.6273;

    // Set at match start based on alliance selection
    public static double targetGoalX = BLUE_GOAL_X;
    public static double targetGoalY = BLUE_GOAL_Y;
}
