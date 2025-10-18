package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class Shooters {

    private DcMotorEx motor1, motor2;
    private PIDWrapper veloPID = new PIDWrapper(new PIDController(Values.p, Values.i, Values.d));

    private double previousTicks;
    private long lastUpdateTime;

    public static double target = 0;
    private int Velocity = 0;


    public Shooters(HardwareMap hardwareMap){

        motor1 = hardwareMap.get(DcMotorEx.class, "Shooter");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reset();

    }
    public void reset(){

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        previousTicks = motor1.getCurrentPosition();
        lastUpdateTime = System.nanoTime();

    }

    public double getPos(){
        return motor1.getCurrentPosition();
    }

    public double getVelo(){

        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9;
        double currentTicks = getPos();
        double deltaTicks = currentTicks - previousTicks;

        double velocityTicksPerSecond = deltaTicks / deltaTime;
        Velocity = (int) (Values.alpha * velocityTicksPerSecond + (1 - Values.alpha) * Velocity);

        previousTicks = currentTicks;
        lastUpdateTime = currentTime;

        return Velocity;
    }

    public void run(){

        double velo = getVelo();

        veloPID.setPID(Values.p, Values.i, Values.d);
        veloPID.setF(Values.f);
        veloPID.setVeloThresh(Values.VeloErrorThresh);
        veloPID.VelocityRun(velo, target, Values.MAX_VELOCITY, Values.MIN_VELOCITY, motor1);

    }

}