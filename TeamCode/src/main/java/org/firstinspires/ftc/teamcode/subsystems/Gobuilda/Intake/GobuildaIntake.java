package org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Intake;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class GobuildaIntake {

    private DcMotorEx intake;

    private double previousTicks;
    private long lastUpdateTime;

    public static double target = 0;
    private int Velocity = 0;

    private PIDWrapper controller = new PIDWrapper(new PDController(Values.p, Values.d));
    public GobuildaIntake(HardwareMap hardwareMap){

        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double getPos(){
        return intake.getCurrentPosition();
    }

    public void setTargetVelo(double val){

        target = val;

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

        controller.setPD(Values.p, Values.d);
        controller.VelocityRun(getVelo(), target, Values.MAX, Values.ZERO, intake);

    }


}
