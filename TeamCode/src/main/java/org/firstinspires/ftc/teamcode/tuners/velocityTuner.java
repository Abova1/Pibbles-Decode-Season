package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
@Configurable
@TeleOp(name="Velocity Tuner", group="tuners")

public class velocityTuner extends OpMode {

    /*
    Values for a 1620 RPM motor:
    P = 0.7
    I = 1
    D = 0.00685
    alpha = 0.09
     */

    public PIDController controller;
    public SimpleMotorFeedforward feedforward;
    public VoltageSensor voltageSensor;
    public DcMotorEx motor1;

    public static double kS = 0, kV= 0, kA = 0;
    public static double kP = 0, kD = 0, kI = 0 /*, f = 0*/;

    public static double target = 0;
    private final double MAX_VELOCITY = 3100;
    private final double MIN_VELOCITY = 0;
    private int previousTicks;
    private long lastUpdateTime;

    public static double alpha = 0;
    private int velocity = 0;

    public static boolean aBoolean = false;

    public static double targetV = 0;
    public static double targetA = 0;


    @Override
    public void init () {

        controller = new PIDController(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        motor1 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        previousTicks = motor1.getCurrentPosition();
        lastUpdateTime = System.nanoTime();

    }

    @Override
    public void loop () {

        if(aBoolean){

            double ff = feedforward.calculate(targetV, targetA);

            motor1.setVelocity(ff);

            telemetry.addData("calculation", ff);
            telemetry.addData("Velocity", motor1.getVelocity());
            telemetry.addData("Target: ", target);

        } else {

            controller.setPID(kP, kI, kD);

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastUpdateTime) / 1e9;
            int currentTicks = motor1.getCurrentPosition();
            int deltaTicks = currentTicks - previousTicks;

            double velocityTicksPerSecond = deltaTicks / deltaTime;
            velocity = (int) (alpha * velocityTicksPerSecond + (1 - alpha) * velocity);

            previousTicks = currentTicks;
            lastUpdateTime = currentTime;

            double PID = controller.calculate(velocity, target);
            double finalOutput = Globals.clamp(target + PID, MAX_VELOCITY, MIN_VELOCITY);

            double velocityError = target - velocity;

            motor1.setVelocity(finalOutput);

            telemetry.addData("SDK Motor Velocity: ", motor1.getVelocity());
            telemetry.addData("Velocity: ", velocity);
            telemetry.addData("Target: ", target);
            telemetry.addData("Output", finalOutput);
            telemetry.addData("Controller Calculation", PID);
            telemetry.addData("Error: ", velocityError);
        }

        telemetry.addData("Motor Current in AMPS", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Voltage Sensor", voltageSensor.getVoltage());
        telemetry.addData("Power: ", motor1.getPower());

        telemetry.update();
    }

}