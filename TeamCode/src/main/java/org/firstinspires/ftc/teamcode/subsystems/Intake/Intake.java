package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class Intake {

    private DcMotorEx intake;

    private PIDWrapper veloPID = new PIDWrapper(new PIDController(Constants.Ip, Constants.Ii, Constants.Id));

    private double previousTicks;
    private long lastUpdateTime;

    public static double target = 0;
    private int Velocity = 0;

    public Intake (HardwareMap hardwareMap){

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reset();

        previousTicks = intake.getCurrentPosition();
        lastUpdateTime = System.nanoTime();

    }

    public void reset(){
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPos(){
        return intake.getCurrentPosition();
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

    public void run(){

        double velo = getVelo();

        veloPID.setPID(Constants.Ip, Constants.Ii, Constants.Ip);
        veloPID.setF(Constants.If);
        veloPID.setVeloThresh(Constants.VeloErrorThresh);
        veloPID.VelocityRun(velo, target, Constants.MAX_VELOCITY, Constants.MIN_VELOCITY, intake);


    }

}
