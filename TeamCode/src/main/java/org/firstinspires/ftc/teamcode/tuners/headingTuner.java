package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.DT;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.DataStorage;


@Config
@TeleOp(name="Heading Tuner", group="tuners")
public class headingTuner extends OpMode {

    //myles poopy head2

    private DcMotorEx motor;
    private PIDController controller;

    public static double p = 0.01875, i = 0.005, d = 0.001475;

    public static double ticksPerRev = 407.785714286;
    public static double targetHeading = 0;
    private double heading;
    private String direction = " ";
    private String positive = "positive";
    private String negative = "negative";

    public static double kTx = 0.07; // how aggressively tx affects targetHeading



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDController(p, i, d);
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void start(){

    }


    @Override
    public void loop() {

        controller.setPID(p, i, d);

        double ticks = motor.getCurrentPosition();
        double degrees = (ticks / ticksPerRev) * 360;

        heading = ((degrees % 360));

        double headingError = targetHeading - heading;

        double power = controller.calculate(heading, targetHeading);

        motor.setPower(-power);



        if(power < 0){
            direction = negative;
        } else if(power > 0){
            direction = positive;
        } else {
            direction = "No Direction";
        }



        telemetry.addData("Calculation", power);
        telemetry.addData("Ticks", ticks);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Heading", heading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("Direction", direction);
        telemetry.update();

    }

    @Override
    public void stop(){


    }

}
