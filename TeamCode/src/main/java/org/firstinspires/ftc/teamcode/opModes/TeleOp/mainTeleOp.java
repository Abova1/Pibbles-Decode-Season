package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DT;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;

@TeleOp (name="Main TeleOp", group="OpModes")
public class mainTeleOp extends LinearOpMode {

    private DT DT;
    private VoltageSensor vs;
    private Controller Driver, Operator;

    private CommandScheduler scheduler;


    @Override
    public void runOpMode() throws InterruptedException {

        vs = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DT = new DT(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            DT.Drive(-Driver.getLy(), Driver.getLx() * 1.1, Driver.getRx());

        }

    }
}