package org.firstinspires.ftc.teamcode.vision.Limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Configurable
@Config
@TeleOp(name="Distance Calc Limelight", group="Vision")
public class DistanceCalculatorLimelight extends OpMode {

    /*
     pipeline 1 is red goal
    pipeline 2 is blue goal
    pipeline 3 is motif
     */



    private Limelight3A limelight;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 10.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 13.50173228;

    // distance from the target to the floor in inches
    double goalHeightInches = 29.5; //29.5" from floor to apriltag



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        limelight = hardwareMap.get(Limelight3A.class, "Limelight");



    }

    @Override
    public void start() {
        limelight.start();

        //starts off with the red goal
        limelight.pipelineSwitch(1);
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

            double targetOffsetAngle_Vertical = ty;
            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            telemetry.addData("Distance", distanceFromLimelightToGoalInches);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }

}
