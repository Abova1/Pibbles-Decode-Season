package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class positionTuner extends OpMode {

    private PIDFController controller;
    private DcMotorEx motor1;
    private VoltageSensor voltageSensor;

    public static double p = 0, d = 0;
    public static double i = 0 , f = 0;
    public static double target = 0;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDFController(p, i, d, f);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        motor1 = hardwareMap.get(DcMotorEx.class,"motor1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop() {

        controller.setPIDF(p,i,d,f);

        double currentPos = motor1.getCurrentPosition();
        double error = target - currentPos;

        double power = controller.calculate(currentPos, target);

        motor1.setPower(power);

        telemetry.addData("Motor Pos: ", currentPos);
        telemetry.addData("Power: ", motor1.getPower());
        telemetry.addData("PID: ", power);
        telemetry.addData("Error: ", error);
        telemetry.addData("Voltage: ", voltageSensor.getVoltage());

        telemetry.update();
    }

}
