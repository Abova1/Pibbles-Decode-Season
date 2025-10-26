package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.TeleHandler;

@TeleOp
public class testingCommands extends OpMode {
    private Shooter shooter;
    private Intake intake;
    private CommandScheduler scheduler;
    private TeleHandler teleHandler;
    private Controller Driver;


    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        scheduler = new CommandScheduler();
        scheduler.clear();

        Driver = new Controller(gamepad1, scheduler);
        teleHandler = new TeleHandler(Driver, scheduler, shooter, intake);

    }

    @Override
    public void loop() {

        scheduler.run();
        teleHandler.TestTeleOp();
        telemetry.update();
    }

}
