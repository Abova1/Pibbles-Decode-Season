package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.DataStorage;
import org.firstinspires.ftc.teamcode.util.Globals;


@Config
@TeleOp
public class headingTuner extends OpMode {

    private DcMotorEx motor;
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;

    public static double ticksPerRev = 28;
    public static double targetHeading = 0;
    public static double MAX = 1;
    public static double MIN = -1;
    private double previousHeading;
    private long heading;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDController(p, i, d);
        motor = hardwareMap.get(DcMotorEx.class, "motor0");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        previousHeading = DataStorage.loadHeading();


    }

    @Override
    public void loop() {

        controller.setPID(p, i, d);

        double ticks = motor.getCurrentPosition();
        double degrees = (ticks / ticksPerRev) * 360;

        heading = (long) (((degrees % 360) + 360) % 360);

        double finalHeading = Globals.clamp(heading, 359, 0.5);


        if(heading >= 359.1){
            targetHeading = 1;
        }
        else if(heading <= 0.1){
            targetHeading = 358;
        }


        double power = controller.calculate(finalHeading, targetHeading);

        power = Globals.clamp(power, MAX, MIN);

        motor.setPower(power);

        double headingError = targetHeading - finalHeading;

        telemetry.addData("Calculation", power);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Heading", heading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("Previous Heading", previousHeading);
        telemetry.update();

    }

    @Override
    public void stop(){

        DataStorage.saveHeading(heading);

    }

}
