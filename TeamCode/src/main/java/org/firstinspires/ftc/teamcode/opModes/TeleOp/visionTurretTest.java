package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Limelight;
import org.firstinspires.ftc.teamcode.util.DataStorage;


@TeleOp(name="Limelight + Turret test", group="Vision")
public class visionTurretTest extends OpMode {

    private Limelight limelight;
    private Turret turret;
    private LLStatus status;
    private LLResult result;
    private double previousHeading;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new Turret(hardwareMap);
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
        telemetry.addData("Heading", turret.getMEHeading());
        telemetry.addData("Last OpMode Heading", previousHeading);
        telemetry.addData("Difference in Headings", limelight.getDiffHeading());
        telemetry.addData("Tx", result.getTx());
        telemetry.addData("Ty", result.getTy());

        telemetry.update();

    }

    @Override
    public void stop(){

        DataStorage.saveHeading(turret.getMEHeading());

    }

}
