package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.Globals;

public class Limelight {

    private Turret turret;
    private Limelight3A limelight;
    private LLResult results;
    private LLStatus status;
    private Sensors sensors;

    private int index;
    private double previousTx;
    private double currentTx;
    private double previousTy;
    private double currentTy;
    private final double targetTx = 0;
    double DiffHeading;


    public Limelight(HardwareMap hardwareMap){

        turret = new Turret(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        sensors = new Sensors(hardwareMap);

        previousTx = 0;
        currentTx = 0;
        previousTy = 0;
        currentTy = 0;

    }


    public void initiate(){

        sensors.initIMU();
        limelight.start();
        limelight.setPollRateHz(200); // 200 updates/sec
        pipeline(Globals.pipelines.Red);//Red Goal

    }

    public void run(){

        results = limelight.getLatestResult();
        status = limelight.getStatus();

        if(index != 3){

            if(results != null && results.isValid()){

                currentTx = results.getTx();
                currentTy = results.getTy();

                double txError = targetTx + currentTx;

                turret.setTargetHeading(turret.getMEHeading() + txError);

                turret.run();

            }
            else {

                if(previousTx > 0){
                    turret.setPower(-0.2);
                }
                else if (previousTx < 0) {
                    turret.setPower(0.2);
                } else {
                    turret.setPower(0.2);
                }

            }

            previousTx = currentTx;
            previousTy = currentTy;

        }

    }

    public void pipeline(Globals.pipelines pipelines){

        index = pipelines.index;

        limelight.pipelineSwitch(index);

    }

    public LLStatus getStatus(){
        return status;
    }

    public LLResult getLatestResults(){
        return limelight.getLatestResult();
    }

    public double getDiffHeading(){
        return DiffHeading;
    }

}
