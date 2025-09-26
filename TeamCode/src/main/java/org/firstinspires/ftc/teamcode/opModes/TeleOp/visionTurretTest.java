package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret.LimeLightMotor;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Limelight;
import org.firstinspires.ftc.teamcode.util.DataStorage;
import org.firstinspires.ftc.teamcode.util.Globals;


@Configurable
@Config
@TeleOp(name="Limelight + Turret test", group="Vision")
public class visionTurretTest extends OpMode {

    private Limelight limelight;
    private LimeLightMotor turret;
    private LLStatus status;
    private LLResult result;
    private double previousHeading;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new LimeLightMotor(hardwareMap);
        limelight = new Limelight(hardwareMap);
        previousHeading = DataStorage.loadHeading();

    }

    @Override
    public void start() {

        limelight.initiate();
        telemetry.setMsTransmissionInterval(175);

    }

    @Override
    public void loop() {

        status = limelight.getStatus();
        result = limelight.getLatestResults();

        limelight.run();

        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        telemetry.addLine();

        telemetry.addData("Motor power", turret.getPower());
        telemetry.addData("Heading", turret.getHeading());
        telemetry.addData("Last Opmode Heading", previousHeading);
        telemetry.addData("Tx", result.getTx());
        telemetry.addData("Ty", result.getTy());

        telemetry.update();

    }

    @Override
    public void stop(){

        DataStorage.saveHeading(turret.getHeading());

    }

}
