package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class LimeLightMotor {

    private DcMotorEx motor;
    private PIDWrapper headingController = new PIDWrapper(new PIDController(Values.p, Values.i, Values.d));
    private Sensors sensors;

    public static double targetHeading;
    private double MAX = 1 ;
    private double MIN = -1;

    public LimeLightMotor(HardwareMap hardwareMap){

        motor = hardwareMap.get(DcMotorEx.class, "LLmotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Unnecessary but i'd like to track the difference in heading
        sensors = new Sensors(hardwareMap);
        sensors.initIMU();
        sensors.resetIMUYaw();

        reset();

    }

    public double getPos(){
        return motor.getCurrentPosition();
    }
    public double getSDKTicksPerRev(){
        return motor.getMotorType().getTicksPerRev();
    }


    public void reset(){

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public double getHeading() {

        double ticks = getPos();
        double degrees = (ticks / Values.TPR) * 360;
        double heading = (long) (((degrees % 360) + 360) % 360);

        return Globals.clamp(heading, 359, 0.5);
    }

    public void run(){

        double currentHeading = getHeading();

        headingController.setPID(Values.p, Values.i, Values.d);

        headingController.TurretRun(currentHeading, targetHeading, MAX, MIN, 1 , motor);

    }


}
