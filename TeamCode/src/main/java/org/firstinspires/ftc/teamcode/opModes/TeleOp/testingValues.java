package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.TeleHandler;

@TeleOp (name="Test OpMode", group="OpModes")
public class testingValues extends LinearOpMode {

    private Controller Driver, Operator;
    private Turret turret;
    private TeleHandler teleHandler;
    private CommandScheduler scheduler;
    private Sensors sensors;


    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = new CommandScheduler();
        scheduler.clear();

        Driver = new Controller(gamepad1, scheduler);
        turret = new Turret(hardwareMap);

        teleHandler = new TeleHandler(Driver, scheduler, turret);

        sensors = new Sensors(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            teleHandler.TeleOp();

            telemetry.addData("Current State", teleHandler.getState());
            telemetry.addData("Command Scheduled", scheduler.currentCommandScheduled());
            telemetry.addData("Voltage", sensors.getVoltage());
            telemetry.addData("Turret Heading",turret.getMEHeading());
            telemetry.addData("Robot Heading", sensors.getIMUHeading(false));

            telemetry.update();

        }

    }

}