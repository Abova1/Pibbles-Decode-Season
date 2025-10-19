package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Custom.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.DT;
import org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Intake.GobuildaIntake;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.TeleHandler;

@TeleOp (name="Pibble TeleOp", group="OpModes")
public class PibbleTeleOp extends LinearOpMode {

    private DT DT;
    private TeleHandler teleHandler;
    private Controller Driver, Operator;
    private CommandScheduler scheduler;

    private Limelight limelight;


    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = new CommandScheduler();
        scheduler.clear();
        Driver = new Controller(gamepad1, scheduler);
        DT = new DT(hardwareMap);
        teleHandler = new TeleHandler(Driver, scheduler);
        limelight = new Limelight(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            limelight.run();

            DT.RCDrive(-(Driver.getLy()), Driver.getLx() * 1.1, Driver.getRx());

            telemetry.update();

        }

    }

}
