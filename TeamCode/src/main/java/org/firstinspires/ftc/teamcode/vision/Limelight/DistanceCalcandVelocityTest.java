package org.firstinspires.ftc.teamcode.vision.Limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
@Configurable
@TeleOp(name="Distance + Velocity Test", group="tuners")

public class DistanceCalcandVelocityTest extends OpMode {

    /*
    Values for a 1620 RPM motor:
    P = 0.7
    I = 1
    D = 0.00685
    alpha = 0.02
     */

    public PIDController controller;
    public SimpleMotorFeedforward feedforward;
    public VoltageSensor voltageSensor;
    public DcMotorEx motor1;
    public Servo hood;

    private Limelight3A limelight;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 10.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 13.50173228;

    // distance from the target to the floor in inches
    double goalHeightInches = 29.5; //29.5" from floor to apriltag

    //    public static double kS = 0, kV= 0, kA = 0;
    public static double kP = 0, kD = 0, kI = 0, F = 0;

    public static double target = 0;
    public static double ServoPos = 0;
    private final double MAX_VELOCITY = 10000;
    private final double MIN_VELOCITY = 0;
    private int previousTicks;
    private long lastUpdateTime;

    public static double Alpha = 0;
    private int velocity = 0;

//    public static boolean aBoolean = false;

//    public static double targetV = 0;
//    public static double targetA = 0;


    @Override
    public void init () {

        controller = new PIDController(kP, kI, kD);
//        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        hood = hardwareMap.get(Servo.class, "hood");

        motor1 = hardwareMap.get(DcMotorEx.class, "shooter");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        previousTicks = motor1.getCurrentPosition();
        lastUpdateTime = System.nanoTime();

    }
    @Override
    public void start() {
        limelight.start();

        //starts off with the red goal
        limelight.pipelineSwitch(1);
    }

    @Override
    public void loop () {

        LLStatus status = limelight.getStatus();




        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);

            double targetOffsetAngle_Vertical = ty;
            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            telemetry.addData("Distance", distanceFromLimelightToGoalInches);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        controller.setPID(kP, kI, kD);

        /*
        Using ticks to calculate an actual velocity since
        .getVelocity likes to go up by 20 ticks/sec which is inconsistent
        when dealing with the PID controller
         */
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9;
        int currentTicks = motor1.getCurrentPosition();
        int deltaTicks = currentTicks - previousTicks;

        double velocityTicksPerSecond = deltaTicks / deltaTime;
        //This is a form of the formula for exponential moving average which smooths the values given
        velocity = (int) (Alpha * velocityTicksPerSecond + (1 - Alpha) * velocity);

        previousTicks = currentTicks;
        lastUpdateTime = currentTime;

        double velocityError = target - velocity;

        double PID = controller.calculate(velocity, target);

        /*
        the reason why it's target PLUS PID is because it has to be a constant value
        and when it fluctuates the PID controller adds more of a target
        */
        double finalOutput = Globals.clamp(target + PID , MAX_VELOCITY, MIN_VELOCITY);

        if(velocityError > 50){
            finalOutput += (F * target);
        }

        //0.1 is down 0.55 is up
        hood.setPosition(ServoPos);

        motor1.setVelocity(finalOutput);

        telemetry.addData("SDK Motor Velocity: ", motor1.getVelocity());
        telemetry.addData("Velocity: ", velocity);
        telemetry.addData("Target: ", target);
        telemetry.addData("Output", finalOutput);
        telemetry.addData("Controller Calculation", PID);
        telemetry.addData("Error: ", velocityError);

        telemetry.addData("Motor Current in AMPS", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Voltage Sensor", voltageSensor.getVoltage());
        telemetry.addData("Power: ", motor1.getPower());

        telemetry.update();
    }

}