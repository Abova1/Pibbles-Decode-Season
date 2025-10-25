package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import static org.firstinspires.ftc.teamcode.tuners.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.tuners.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.tuners.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.tuners.pedroPathing.Tuning.stopRobot;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DT;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.tuners.pedroPathing.Constants;

import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.TeleHandler;


@Configurable
@TeleOp (name = "MylesAlignTurretTele", group = "TeleOp")
public class MylesAlignTurretTeleOpTest extends OpMode {

    private Shooter shooter;
    private CommandScheduler scheduler;
    private Controller Driver;
    private TeleHandler teleHandler;
    private ShooterSubsystem shooterSubsystem;

    private Turret turret;

    public static Follower follower;
    private TelemetryManager telemetryM;

    public boolean automatedDrive;

    public static int testposition;
    public static double P = 0;

    private Supplier<PathChain> ParkpathChain;

    private DT drive;
    public static Pose autoEndPose;

    public void init() {

        shooter = new Shooter(hardwareMap);

        scheduler = new CommandScheduler();

        Driver = new Controller(gamepad1, scheduler);

        teleHandler = new TeleHandler(Driver, scheduler, shooter);

        turret = new Turret(hardwareMap);


        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        drive = new DT(hardwareMap);

        telemetry.addData("Status", "Initialized");
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(23.629279390557905,120.3707206094421, Math.toRadians(0));
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
        teleHandler.TeleOp();
        telemetry.addLine(scheduler.currentCommandScheduled());


        if (!automatedDrive) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        /*
        if (gamepad1.aWasPressed()) { //MAKE THIS BUTTON ANYTHING! WILL AUTO PARK I THINK
            follower.followPath(ParkpathChain.get());
            automatedDrive = true;
        }
         */
//        shooterSubsystem.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), true, telemetry);

        turret.pinpointTurret(follower.getHeading());


        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Robot Heading", turret.getRobotHeading());
        telemetry.addData("Turret Offset", turret.getTurretOffset());
        telemetry.addData("TurretHeading", turret.getMEHeading());

        telemetryM.update(telemetry);

        telemetry.update();
    }
    public void stop() {
        autoEndPose = follower.getPose();
    }
}
