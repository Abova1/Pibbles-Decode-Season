package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class Shooter {

    private DcMotorEx shooter;
    private PIDWrapper veloPID = new PIDWrapper(new PIDController(Constants.Sp, Constants.Si, Constants.Sd));

    private double previousTicks;
    private long lastUpdateTime;

    public static double target = 0;
    private int Velocity = 0;


    public Shooter(HardwareMap hardwareMap){

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        reset();

        previousTicks = shooter.getCurrentPosition();
        lastUpdateTime = System.nanoTime();

    }
    public void reset(){
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPos(){
        return shooter.getCurrentPosition();
    }

    public static void setShooterTarget(double newTarget){
        target = newTarget;
    }

    public double getVelo(){

        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9;
        double currentTicks = getPos();
        double deltaTicks = currentTicks - previousTicks;

        double velocityTicksPerSecond = deltaTicks / deltaTime;
        Velocity = (int) (Constants.alpha * velocityTicksPerSecond + (1 - Constants.alpha) * Velocity);

        previousTicks = currentTicks;
        lastUpdateTime = currentTime;

        return Velocity;
    }

    public double getRPM(){
        return (getVelo() /Constants.TPR) * 60;

    }

    public void run(){

        double velo = getVelo();

        veloPID.setPID(Constants.Sp, Constants.Si, Constants.Sd);
        veloPID.setF(Constants.Sf);
        veloPID.setVeloThresh(Constants.VeloErrorThresh);
        veloPID.VelocityRun(velo, target, Constants.MAX_VELOCITY, Constants.MIN_VELOCITY, shooter);

    }

}