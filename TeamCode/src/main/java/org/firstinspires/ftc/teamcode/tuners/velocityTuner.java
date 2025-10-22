package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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


    public PIDController controller;
    public VoltageSensor voltageSensor;
    public DcMotorEx motor1;
    public static double kP = 0, kD = 0, kI = 0, F = 0;

    public static double TargetRPM = 0;
    private int previousTicks;
    private long lastUpdateTime;
    public static double Alpha = 0;
    private int velocity = 0;
    public static boolean MOTOR_POWER = false;

    @Override
    public void init () {

        controller = new PIDController(kP, kI, kD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        motor1 = hardwareMap.get(DcMotorEx.class, "shooter");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        previousTicks = motor1.getCurrentPosition();
        lastUpdateTime = System.nanoTime();

    }

    @Override
    public void loop () {

        controller.setPID(kP, kI, kD);

        if(MOTOR_POWER){
            motor1.setPower(1);
            double RPM = (motor1.getVelocity() / Globals.MOTOR_TICKS.RPM_1620.ticksPerRev) * 60;

            telemetry.addData("RPM", RPM);
        } else {

            /*
            Using ticks to calculate an actual velocity since
            .getVelocity likes to go up by 20 ticks/sec which is inconsistent
            when dealing with the PID controller
            */
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastUpdateTime) / 1e9;
            int currentTicks = motor1.getCurrentPosition();
            int deltaTicks = currentTicks - previousTicks;

            int velocityTicksPerSecond = (int) (deltaTicks / deltaTime);
            //This is a form of the formula for exponential moving average which smooths the values given
            velocity = (int) (Alpha * velocityTicksPerSecond + (1 - Alpha) * velocity);

            double RPM = (velocity / Globals.MOTOR_TICKS.RPM_1620.ticksPerRev) * 60;

            double targetVelocityTicksPerSecond = (TargetRPM / 60.0) * Globals.MOTOR_TICKS.RPM_1620.ticksPerRev;


            previousTicks = currentTicks;
            lastUpdateTime = currentTime;


            double velocityError = targetVelocityTicksPerSecond - velocity;
            double RPMerror = TargetRPM - RPM;

            double PID = controller.calculate(velocity, targetVelocityTicksPerSecond);

            /*
            the reason why it's target PLUS PID is because it has to be a constant value
            and when it fluctuates the PID controller adds more of a target
            */
            double finalOutput = targetVelocityTicksPerSecond + PID;

            if(velocityError > 50){
                finalOutput += (F * targetVelocityTicksPerSecond);
            }

            motor1.setVelocity(finalOutput);

            telemetry.addData("Output", finalOutput);
            telemetry.addData("Controller Calculation", PID);
            telemetry.addData("Error: ", velocityError);
            telemetry.addData("RPM ERROR:", RPMerror);
            telemetry.addData("RPM", RPM);
        }

      


        telemetry.addData("SDK Motor Velocity: ", motor1.getVelocity());
        telemetry.addData("Velocity: ", velocity);
        telemetry.addData("Motor Current in AMPS", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Power: ", motor1.getPower());

        telemetry.update();
    }

}