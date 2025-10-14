package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensors;

public class DT {

    private DcMotorEx FL, BL, FR, BR;
    private Sensors sensors;



    public DT (HardwareMap hardwareMap) {

        sensors = new Sensors(hardwareMap);
        sensors.initIMU();

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void FCDrive(double y, double x, double rx){

        double botHeading = sensors.getIMUHeading(true);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);

    }

    public void RCDrive(double y, double x, double rx) {
        double voltage = sensors.getVoltage();
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