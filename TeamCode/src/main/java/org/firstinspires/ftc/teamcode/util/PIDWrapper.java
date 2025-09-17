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

    public double PIDCalc(double current, double target){
        return PID.calculate(current, target);
    }

    public double PIDFCalc(double current, double target){
        return PIDF.calculate(current, target);
    }

    public void setPIDF(double p, double i, double d, double f){
        PIDF.setPIDF(p, i, d, f);
    }

    public void setPID(double p, double i, double d){
        PID.setPID(p, i, d);
    }

    public void PositionRun(double current, double target, double MAX, double MIN, DcMotorEx... motors){

        double power = PIDCalc(current, target);
        double finalOutput = Globals.clamp(power, MAX, MIN);

        for(DcMotorEx motor : motors){
            motor.setPower(finalOutput);
        }
    }

    public void VelocityRun(double current, double target, double MAX, double MIN, DcMotorEx... motors) {

        double pid = PIDCalc(current, target);
        double finalOutput = Globals.clamp(target + pid, MAX, MIN);

        for (DcMotorEx motor : motors) {
            motor.setVelocity(finalOutput);
        }
    }

}