package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class DT {

    private VoltageSensor sensor;
    private DcMotorEx FL, BL, FR, BR;

    public DT (HardwareMap hardwareMap, VoltageSensor sensor) {

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        this.sensor = sensor;

    }

    public void Drive (double y, double x, double rx) {
        double voltage = sensor.getVoltage();
        double shorterOutPut = 0.75;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        if(voltage > 13){
            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);
        }
        else{
            FL.setPower(frontLeftPower * shorterOutPut);
            BL.setPower(backLeftPower * shorterOutPut);
            FR.setPower(frontRightPower * shorterOutPut);
            BR.setPower(backRightPower * shorterOutPut);
        }

    }

}