package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class PIDWrapper {

    private PIDFController PIDF;
    private PIDController PID;
    private PDController PD;
    private double F;
    private double Thresh;
    private boolean Inversed = false;


    public PIDWrapper(PIDFController pidf){
        this.PIDF = pidf;
    }
    public PIDWrapper(PIDController pid) {
        this.PID = pid;
    }
    public PIDWrapper(PDController pd){
        this.PD = pd;
    }


    public double calc(double current, double target){

        if(PIDF != null){
            return PIDF.calculate(current, target);
        }
        else if(PID != null){
            return PID.calculate(current, target);
        }
        else if(PD != null){
            return PD.calculate(current, target);
        }

        return 0;
    }

    public void setPIDF(double p, double i, double d, double f){
        PIDF.setPIDF(p, i, d, f);
    }

    public void setPID(double p, double i, double d){
        PID.setPID(p, i, d);
    }
    public void setPD(double p, double d){
        PD.setP(p);
        PD.setD(d);
    }
    public void setF(double f){
        F = f;
    }

    public void setVeloThresh(double thresh){
        Thresh = thresh;
    }
    public void setInverse(boolean inversed){
        Inversed = inversed;
    }

    public void PositionRun(double current, double target, DcMotorEx... motors){

        double power = calc(current, target);

        for(DcMotorEx motor : motors){

            if(Inversed) {
                motor.setPower(-power);
            } else {
                motor.setPower(power);
            }

        }

    }

    public void VelocityRun(double current, double target, double MAX, double MIN, DcMotorEx... motors) {

        double pid = calc(current, target);
        double finalOutput = Globals.clamp(target + pid, MAX, MIN);

        if((target - current) > Thresh){
            finalOutput += F * target;
        }

        for (DcMotorEx motor : motors) {
            motor.setVelocity(finalOutput);
        }

    }

}