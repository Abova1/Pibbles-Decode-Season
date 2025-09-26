package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Sensors {

    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;
    private ColorSensor colorSensor;
    private DigitalChannel absEncoder;


    public Sensors(HardwareMap hardwareMap){

        imu = hardwareMap.get(IMU.class, "imu");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        //Don't use until needed
//        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

    }

    public void initIMU(){

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward

        imu.initialize(parameters);

    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetIMUYaw(){
        imu.resetYaw();
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }



}
