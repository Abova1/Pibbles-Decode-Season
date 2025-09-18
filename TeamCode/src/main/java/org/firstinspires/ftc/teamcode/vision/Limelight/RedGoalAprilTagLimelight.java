package org.firstinspires.ftc.teamcode.vision.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Red Goal AprilTag Limelight", group="Vision")
public class RedGoalAprilTagLimelight extends OpMode {

    private Limelight3A limelight;
    private DcMotorEx LLmotor;

    private double prevError = 0;
    private double lastTime = 0;


    /*
     pipeline 1 is red goal
    pipeline 2 is blue goal
    pipeline 3 is motif
     */
    @Override
    public void init() {

        LLmotor = hardwareMap.get(DcMotorEx.class, "LLmotor");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        LLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        limelight.start();
        limelight.pipelineSwitch(1);
        lastTime = getRuntime();

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
        double now = getRuntime();
        double dt = Math.max(0.001, now - lastTime);

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            double kP = 0.01;
            double kD = 0.002;// proportional gain (tune this)
            double deadband = 0.02;    // don’t bother running motor if |power| < 0.02
            double maxPower = 0.3;
            double txTolerance = 1.5;

            double error = tx;
            double derivative = (error - prevError) / dt;
            double power = kP * error + kD * derivative;

            power = Math.max(-maxPower, Math.min(maxPower, power));
            if (Math.abs(tx) < txTolerance) {
                power = 0;
            }
            if (Math.abs(power) < deadband) {
                power = 0;
            }
            LLmotor.setPower(power);
            prevError = error;
            lastTime = now;
            telemetry.addData("Motor power", power);
        } else {
            LLmotor.setPower(0);
            prevError = 0;
            lastTime = now;
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
