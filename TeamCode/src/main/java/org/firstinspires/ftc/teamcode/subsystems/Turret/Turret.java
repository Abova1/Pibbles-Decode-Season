package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class Turret {

    private DcMotorEx motor;
    private AnalogInput Encoder;

    private PIDWrapper headingController = new PIDWrapper(new PIDController(Values.p, Values.i, Values.d));

    private Sensors sensors;

    public static double targetHeading;
    private double MAX = 360;
    private double MIN = 1;

    public Turret(HardwareMap hardwareMap){

        motor = hardwareMap.get(DcMotorEx.class, "motor0");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        Encoder = hardwareMap.get(AnalogInput.class, "Turret Encoder");

        sensors = new Sensors(hardwareMap);
        sensors.initIMU();
        sensors.resetIMUYaw();

        reset();

    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public double getPower(){
        return motor.getPower();
    }

    public double getPos(){
        return motor.getCurrentPosition();
    }

    public void reset(){

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    //ANALOG ENCODER VALUES
    public double getAnalogHeading() {

        double voltage = Encoder.getVoltage();

        double degrees = (voltage / 3.3) * 360;

        return (((degrees % 360) + 360) % 360);
    }

    //ME is MOTOR ENCODER
    public double getMEHeading() {

        double TPR;

        if(Values.GearRatio != 0){
            TPR = Values.TPR * Values.GearRatio;
        } else {
            TPR = Values.TPR;
        }

        double ticks = getPos();
        double degrees = (ticks / TPR) * 360;

        return (((degrees % 360) + 360) % 360);

    }


    public void setTargetHeading (double degrees){
        targetHeading = Globals.clamp(degrees, 358, 0.5);
    }

    public void run(){

        /*
        This sets a desired field heading (ex. PP's Field Map)
        We can change this around based on how we start or just not use it at all
         */

        double fieldHeading = 45;
        double imuHeading = sensors.getIMUHeading(false);
        double turretOffset = ((fieldHeading - imuHeading) + 360) % 360;

        headingController.setPID(Values.p, Values.i, Values.d);

        setTargetHeading(turretOffset);

        headingController.TurretRun(getMEHeading(), targetHeading, MAX, MIN, 0.5 , motor);

    }

}
