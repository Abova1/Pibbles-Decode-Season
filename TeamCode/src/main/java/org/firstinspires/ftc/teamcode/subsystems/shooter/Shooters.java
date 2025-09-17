package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class Shooters {

    private DcMotorEx motor1, motor2;
    private PIDWrapper veloPID = new PIDWrapper(new PIDController(Values.p, Values.i, Values.d));

    private int previousTicks;
    private long lastUpdateTime;

    private final double alpha = Values.alpha;
    private double target = 0;
    private int Velocity = 0;


    public Shooters(HardwareMap hardwareMap){

        motor1 = hardwareMap.get(DcMotorEx.class, "motor0");
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
        int currentTicks = (int) getPos();
        int deltaTicks = currentTicks - previousTicks;

        double velocityTicksPerSecond = deltaTicks / deltaTime;
        Velocity = (int) (alpha * velocityTicksPerSecond + (1 - alpha) * Velocity);

        previousTicks = currentTicks;
        lastUpdateTime = currentTime;

        return Velocity;
    }

    public void run(){

        veloPID.setPID(Values.p, Values.i, Values.d);
        veloPID.VelocityRun(getVelo(), target, Values.MAX_VELOCITY, Values.MIN_VELOCITY, motor1);

    }

    //Commands go under


}