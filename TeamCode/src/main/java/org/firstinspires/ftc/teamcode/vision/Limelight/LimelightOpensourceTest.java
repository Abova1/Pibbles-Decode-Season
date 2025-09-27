package org.firstinspires.ftc.teamcode.vision.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LL Open source test", group="Vision")
public class LimelightOpensourceTest extends OpMode {

    private Limelight3A limelight;
    private DcMotor LLmotor;

    /*
    pipeline 1 is red goal
    pipeline 2 is blue goal
    pipeline 3 is motif
     */
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        LLmotor = hardwareMap.get(DcMotor.class, "LLmotor");
        limelight.pipelineSwitch(1);
    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);

            if (tx > 5) {
                LLmotor.setPower(0.1);
            } else if (tx < -5) {
                LLmotor.setPower(-0.1);
            } else {
                LLmotor.setPower(0);
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
