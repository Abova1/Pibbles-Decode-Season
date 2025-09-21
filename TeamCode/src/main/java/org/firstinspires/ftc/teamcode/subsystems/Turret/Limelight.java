package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight {

    private Limelight3A limelight;
    private LLStatus status;
    private LLResult results;
    private Telemetry telemetry;

    private double previousTx;
    private double currentTx;
    private double previousTy;
    private double currentTy;
    private final double targetTx = 0;
    private double txError;




    public Limelight(HardwareMap hardwareMap){

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.start();

        previousTx = 0;
        currentTx = 0;
        previousTy = 0;
        currentTy = 0;
        txError = 0;

    }

    public void mode(double index){

        if(index == Values.Red){
            limelight.pipelineSwitch(Values.Red);
        } else if (index == Values.Blue){
            limelight.pipelineSwitch(Values.Blue);
        } else if (index == Values.Motif){
            limelight.pipelineSwitch(Values.Motif);
        }

    }

    public double txToTargetHeading(){

        return 0; // unknown

    }

    public void run(){

        status = limelight.getStatus();
        results = limelight.getLatestResult();

        if(results != null && results.isValid()){

            currentTx = results.getTx();
            currentTy = results.getTy();

            txError = targetTx + currentTx;

            previousTx = currentTx;
            previousTy = currentTy;

        }
        else {

            // no clue what to do;

        }

    }




}
