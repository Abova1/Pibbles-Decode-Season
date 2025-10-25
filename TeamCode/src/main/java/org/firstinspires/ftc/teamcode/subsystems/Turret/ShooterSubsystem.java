package org.firstinspires.ftc.teamcode.subsystems.Turret;

import static java.lang.Math.atan;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class ShooterSubsystem {
    public static double turretOffsetY = -1.750;
    public static double turretOffsetX  = 0;
    public static double blueGoalX = 12;
    public static double blueGoalY = 134;
    public static double redGoalX  = 136;
    public static double redGoalY  = 134;

    private DcMotorEx turret = null;
    public static double tSlope = 1.13273809524;

    public static int pos = 0;

    public double P;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTurretPosition(int pos) {
        turret.setPositionPIDFCoefficients(P);
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(-1);
    }


    public void setP(double p){
        P = p;
    }

    public void update() {
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry) {
        double headingDeg = Math.toDegrees(heading);

        x = turretOffsetX + x;
        y = turretOffsetY + y;

        // ---- Pick correct goal ----
        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        double angleToGoal = Math.toDegrees(Math.atan2(goalX - x, goalY - y));

        double turretAngle = angleToGoal + headingDeg - 90;

        int targetTicks = (int) (tSlope * turretAngle);

        final int TURRET_MIN = -550;
        final int TURRET_MAX = 550;
        targetTicks = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetTicks));

        pos = targetTicks;

        double distance = Math.hypot(goalX-x, goalY-y);

        setTurretPosition(pos);
    }



    public int getPos() {
        return turret.getCurrentPosition();
    }
    public void telemetry() {
    }
}