package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.DT;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
@Configurable
@TeleOp
public class TestTeleNoChamber extends LinearOpMode {

    DcMotorEx turret, Intake, Shooter;

    Servo hood;

    DT drive;

    public static double kTx = 0.07; // how aggressively tx affects targetHeading

    private PIDController TurretController, ShooterController;
    private Limelight3A limelight;

    public static double Tp = 0.01875, Ti = 0.005, Td = 0.001475;

    public static double ticksPerRev = 407.785714286;
    public static double targetHeading = 0;
    public static double MAX = 1;
    public static double MIN = -1;

    private double heading;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 10.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 13.50173228;

    // distance from the target to the floor in inches
    double goalHeightInches = 29.5; //29.5" from floor to apriltag
    public static double Sp = 0.001, Sd = 0.002, Si = 0, F = 0.001;


    public static double target = 0;
    private final double MAX_VELOCITY = 10000;
    private final double MIN_VELOCITY = 0;
    private int previousTicks;
    private long lastUpdateTime;

    public double distanceFromLimelightToGoalInches;

    public double ServoPosCalculation;

    public static double Alpha = 0.3;
    private int velocity = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TurretController = new PIDController(Tp, Ti, Td);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(150);

        ShooterController = new PIDController(Sp, Si, Sd);
        Shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hardwareMap.get(Servo.class, "hood");

        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new DT(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){

            drive.RCDrive(-(gamepad1.left_stick_y), gamepad1.left_stick_x, gamepad1.right_stick_x);

            LLStatus status = limelight.getStatus();

            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            TurretController.setPID(Tp, Ti, Td);

            double ticks = turret.getCurrentPosition();
            double degrees = (ticks / ticksPerRev) * 360;

            heading = ((degrees % 360));
            //min is -250 max is 135

            // Safety thresholds
            double upperThreshold = 122.5;
            double lowerThreshold = -237.5;

            double headingError = targetHeading - heading;

            if (heading > upperThreshold && headingError > 0) {
                // About to go past upper limit, unwrap clockwise
                targetHeading -= 360;
            } else if (heading < lowerThreshold && headingError < 0) {
                // About to go past lower limit, unwrap counter-clockwise
                targetHeading += 360;
            }

            double power = TurretController.calculate(heading, targetHeading);
            turret.setPower(-power);



            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);

                targetHeading -= kTx * tx;

                double targetOffsetAngle_Vertical = ty;
                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                //calculate distance
                distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


            } else {

                if(gamepad1.right_bumper){
                    targetHeading -= 1.5;
                } else if(gamepad1.left_bumper){
                    targetHeading += 1.5;
                }

            }

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastUpdateTime) / 1e9;
            int currentTicks = Shooter.getCurrentPosition();
            int deltaTicks = currentTicks - previousTicks;

            double velocityTicksPerSecond = deltaTicks / deltaTime;
            //This is a form of the formula for exponential moving average which smooths the values given
            velocity = (int) (Alpha * velocityTicksPerSecond + (1 - Alpha) * velocity);

            previousTicks = currentTicks;
            lastUpdateTime = currentTime;



            double velocityError = target - velocity;

            double PID = ShooterController.calculate(velocity, target);

            double finalOutput = Globals.clamp(target + PID , MAX_VELOCITY, MIN_VELOCITY);

            if(velocityError > 50){
                finalOutput += (F * target);
            }

            //0.1 is down 0.55 is up
            ServoPosCalculation = ((-0.00391524) * (distanceFromLimelightToGoalInches)) + 0.696695;

            ServoPosCalculation = Globals.clamp(ServoPosCalculation, 0.55, 0.1);

            hood.setPosition(ServoPosCalculation);

            Shooter.setVelocity(finalOutput);

            if(gamepad1.a){
                Intake.setPower(1);
            }else {
                Intake.setPower(0);
            }

            telemetry.update();
        }
    }
}
