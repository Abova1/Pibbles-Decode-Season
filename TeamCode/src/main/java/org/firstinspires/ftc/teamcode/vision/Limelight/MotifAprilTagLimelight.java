package org.firstinspires.ftc.teamcode.vision.Limelight;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
@TeleOp(name="Motif AprilTag Limelight", group="Vision")
public class MotifAprilTagLimelight extends OpMode {

    private Limelight3A limelight;

    /*
    pipeline 1 is red goal
    pipeline 2 is blue goal
    pipeline 3 is motif

     */
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
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
            String motiffound = "NOT FOUND";
            String motif = "no pattern yet";
            int ApriltagID = -1;
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                ApriltagID = fr.getFiducialId();
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
            if (ApriltagID == 21) {
                motiffound = "FOUND";
                motif = "GPP";
                telemetry.addData("Motif Found: ", motiffound);
                telemetry.addData("motif ", motif);
            }
            if (ApriltagID == 22) {
                motiffound = "FOUND";
                motif = "PGP";
                telemetry.addData("Motif Found: ", motiffound);
                telemetry.addData("motif ", motif);
            }
            if (ApriltagID == 23) {
                motiffound = "FOUND";
                motif = "PPG";
                telemetry.addData("Motif Found: ", motiffound);
                telemetry.addData("motif ", motif);
            }
            limelight.close();
        }
    }
}
