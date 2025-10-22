package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.bylazar.gamepad.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.TeleHandler;

@TeleOp
public class TestTeleOp extends LinearOpMode {

    private Shooter shooter;
    private CommandScheduler scheduler;
    private Controller Driver;
    private TeleHandler teleHandler;


    @Override
    public void runOpMode() throws InterruptedException {
        //INIT ANYTHING

        shooter = new Shooter(hardwareMap);

        scheduler = new CommandScheduler();

        Driver = new Controller(gamepad1, scheduler);
        teleHandler = new TeleHandler(Driver, scheduler, shooter);

        waitForStart();
        if(isStopRequested()) return;

        while (opModeIsActive()){
            //THE LOOP

            teleHandler.TeleOp();

            telemetry.addLine(scheduler.currentCommandScheduled());
            telemetry.update();

        }

    }
}
