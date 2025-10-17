package org.firstinspires.ftc.teamcode.subsystems.Custom.Indexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensors;

public class Intake {

    private DcMotorEx motor;
    private Sensors sensors;

    public int[] order = {1, 2, 3};


    public Intake (HardwareMap hardwareMap){

        sensors = new Sensors(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "Intake");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void run(){

    }

}
