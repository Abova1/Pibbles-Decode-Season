package org.firstinspires.ftc.teamcode.subsystems.Turret;

import org.firstinspires.ftc.teamcode.util.Globals;

class Constants {

    public static double blueGoalX = 12;
    public static double blueGoalY = 134;
    public static double redGoalX  = 136;
    public static double redGoalY  = 134;

    public static double turretOffsetY = 4.09;// how far our turret is from middle of robot in y direction
    public static double turretOffsetX  = 0; // how far our turret is from middle of robot in x direction (positive x&y and negative x&y is normal, look from top down view of robot)

    public static double tSlope = 1.13273809524;
    public static double Tp = 0.057, Ti = 0, Td = 0.00055;
    public static double kTx = 0.07; // how aggressively tx affects targetHeading

    public static double GearRatio = 3.92857142857;
    public static double TPR = Globals.MOTOR_TICKS.RPM_1620.ticksPerRev;

    public static double upperThreshold = 190;
    public static double lowerThreshold = -265;

    /*======================================|Hood|============================================================ */

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double limelightMountAngleDegrees = 10.0;

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightInches = 13.50173228;
    public static double goalHeightInches = 29.5; //29.5" from floor to apriltag
    public static double hoodUp = 0.55;
    public static double hoodDown = 0.1;


}
