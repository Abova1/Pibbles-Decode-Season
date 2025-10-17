package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Intake.GobuildaIntake;
import org.firstinspires.ftc.teamcode.subsystems.DT;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;

@TeleOp (name="GoBuilda TeleOp", group="OpModes")
public class GoBuildaTeleOp extends LinearOpMode {

    private DT DT;
    private GobuildaIntake intake;
    private TeleHandler teleHandler;
    private Controller Driver, Operator;
    private CommandScheduler scheduler;


    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = new CommandScheduler();
        scheduler.clear();

        Driver = new Controller(gamepad1, scheduler);

        DT = new DT(hardwareMap);


        teleHandler = new TeleHandler(Driver, scheduler, intake);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            teleHandler.TeleOp();

            DT.RCDrive(-(Driver.getLy()), Driver.getLx() * 1.1, Driver.getRx());

            telemetry.update();

        }

    }
}