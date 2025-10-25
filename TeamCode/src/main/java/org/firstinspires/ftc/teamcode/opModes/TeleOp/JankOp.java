package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Chamber.Chamber;
import org.firstinspires.ftc.teamcode.subsystems.DT;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.tuners.pedroPathing.Constants;

import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.TeleHandler;


@Configurable
@TeleOp (name = "Full Jank Tele", group = "TeleOp")
public class JankOp extends OpMode {

    private Shooter shooter;
    private CommandScheduler scheduler;
    private Controller Driver;
    private TeleHandler teleHandler;
    private Turret turret;



    public static Follower follower;
    private TelemetryManager telemetryM;

    public boolean automatedDrive;

    private Supplier<PathChain> ParkpathChain;

    public static Pose autoEndPose;


    private DcMotorEx Intake, Shooter;
    private Servo hood, chamber;

    private PIDController ShooterController;

    public double Sp = 0.001, Sd = 0.002, Si = 0, F = 0.001;


    public double target = 0;
    private final double MAX_VELOCITY = 10000;
    private final double MIN_VELOCITY = 0;
    private int previousTicks;
    private long lastUpdateTime;

    public double distanceFromLimelightToGoalInches;

    public double ServoPosCalculation;

    public double Alpha = 0.3;
    private int velocity = 0;



    public void init() {

        ShooterController = new PIDController(Sp, Si, Sd);
        Shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hardwareMap.get(Servo.class, "hood");
        chamber = hardwareMap.get(Servo.class, "chamber");

        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = new Turret(hardwareMap);


        telemetry.addData("Status", "Initialized");
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(23.629279390557905,120.3707206094421, Math.toRadians(180));
        follower.setStartingPose(autoEndPose == null ? startPose : autoEndPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //----------------------AUTO-PARKING----------------------------------
        ParkpathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.5, 33))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
    }
    public void start() {
        follower.startTeleopDrive();

    }
    public void loop() {
        follower.update();
        telemetryM.update();



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
        ServoPosCalculation = ((-0.00391524) * (turret.getPDistance())) + 0.696695;

        ServoPosCalculation = Globals.clamp(ServoPosCalculation, 0.55, 0.1);

        hood.setPosition(ServoPosCalculation);

        Shooter.setVelocity(finalOutput);

        if(gamepad1.dpad_down){
            target = 2000;
        } else if (gamepad1.dpad_up) {
            target = 2580;
        } else if (gamepad1.dpad_right) {
            target = 0;
        }

        if(gamepad1.a){
            Intake.setPower(1);
        }else {
            Intake.setPower(0);
        }

        telemetry.update();


        if (!automatedDrive) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        /*
        if (gamepad1.aWasPressed()) { //MAKE THIS BUTTON ANYTHING! WILL AUTO PARK I THINK
            follower.followPath(ParkpathChain.get());
            automatedDrive = true;
        }
         */
        turret.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), true);




        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Turret Pos: ", turret.getTurretAngle());
        telemetry.addData("Encoder Heading Pos: ", turret.getMEHeading());
        telemetry.addData("Turret Encoder Pos: ", turret.getTurretPos());
        telemetry.addData("Pedro Distance", turret.getPDistance());
        telemetry.update();
    }
    public void stop() {
        autoEndPose = null; //CHANGE WHEN WE ADD AN AUTONOMOUS TO BELOW!
        //autoEndPose = follower.getPose();
    }
}
