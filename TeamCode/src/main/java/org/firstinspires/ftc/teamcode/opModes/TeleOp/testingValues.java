package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooters;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.TeleHandler;

@TeleOp (name="Test OpMode", group="OpModes")
public class testingValues extends LinearOpMode {

    private Controller Driver, Operator;
    private Shooters shooter;
    private TeleHandler teleHandler;
    private CommandScheduler scheduler;
    private VoltageSensor vs;

    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = new CommandScheduler();

        scheduler.clear();

        Driver = new Controller(gamepad1, scheduler);

        shooter = new Shooters(hardwareMap);

        teleHandler = new TeleHandler(Driver, scheduler, shooter);

        vs = hardwareMap.get(VoltageSensor.class, "Control Hub");

        telemetry.setMsTransmissionInterval(175);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            scheduler.run();
            teleHandler.TeleOp();

            telemetry.addData("Current State", teleHandler.getState());
            telemetry.addData("Command Scheduled", scheduler.currentCommandScheduled());
            telemetry.addData("Voltage", vs.getVoltage());
            telemetry.addData("Shooter Velocity", shooter.getVelo());
            telemetry.update();

        }

    }

}