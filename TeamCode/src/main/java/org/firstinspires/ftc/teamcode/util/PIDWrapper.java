package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class PIDWrapper {

    private PIDFController PIDF;
    private PIDController PID;


    public PIDWrapper(PIDFController pidf){
        this.PIDF = pidf;
    }
    public PIDWrapper(PIDController pid) {
        this.PID = pid;
    }

    public double calc(double current, double target){

        if(PIDF != null){
            return PIDF.calculate(current, target);
        }
        else if(PID != null){
            return PID.calculate(current, target);
        }

        return 0;
    }

    public void setPIDF(double p, double i, double d, double f){
        PIDF.setPIDF(p, i, d, f);
    }

    public void setPID(double p, double i, double d){
        PID.setPID(p, i, d);
    }

    public void PositionRun(double current, double target, DcMotorEx... motors){

        double power = calc(current, target);

        for(DcMotorEx motor : motors){
            motor.setPower(power);
        }

    }

    public void VelocityRun(double current, double target, double MAX, double MIN, DcMotorEx... motors) {

        double pid = calc(current, target);
        double finalOutput = Globals.clamp(target + pid, MAX, MIN);

        for (DcMotorEx motor : motors) {
            motor.setVelocity(finalOutput);
        }

    }

}