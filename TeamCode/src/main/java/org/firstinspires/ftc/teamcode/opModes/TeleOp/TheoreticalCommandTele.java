package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Chamber.Chamber;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.tuners.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.TeleHandler;

import java.util.function.Supplier;

@TeleOp (name = "Command TeleOp", group = "TeleOp")
public class TheoreticalCommandTele extends OpMode {

    private Shooter shooter;
    private Turret turret;
    private Intake intake;
    private Chamber chamber;
    private CommandScheduler scheduler;
    private Controller Driver;
    private TeleHandler teleHandler;

    public static Follower follower;

    public boolean automatedDrive;

    private Supplier<PathChain> ParkpathChain;

    public static Pose autoEndPose;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        chamber = new Chamber(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);

        scheduler = new CommandScheduler();
        scheduler.clear();

        Driver = new Controller(gamepad1, scheduler);
        teleHandler = new TeleHandler(Driver, scheduler, shooter, chamber, turret, intake);

        telemetry.addData("Status", "Initialized");
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(23.629279390557905,120.3707206094421, Math.toRadians(180));
        follower.setStartingPose(autoEndPose == null ? startPose : autoEndPose);
        follower.update();

        //----------------------AUTO-PARKING----------------------------------
        ParkpathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.5, 33))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        if (!automatedDrive) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        chamber.setChamberPos(0.875);
        scheduler.run();
        teleHandler.TeleOp();
        turret.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), true);

        telemetry.update();
    }

    @Override
    public void stop(){
        autoEndPose = null; //CHANGE WHEN WE ADD AN AUTONOMOUS TO BELOW!
        //autoEndPose = follower.getPose();
    }

}
