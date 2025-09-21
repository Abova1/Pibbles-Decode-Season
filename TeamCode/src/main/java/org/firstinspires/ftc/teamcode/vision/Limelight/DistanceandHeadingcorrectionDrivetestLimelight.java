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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@Config
@TeleOp(name="Distance + Heading AprilTag Limelight", group="Vision")
public class DistanceandHeadingcorrectionDrivetestLimelight extends OpMode {

    /*
     pipeline 1 is red goal
    pipeline 2 is blue goal
    pipeline 3 is motif
     */
    public static double Dp = 0.1; //DRIVE P TUNE THIS
    public static double Kp = -0.1; //HEADING P TUNE THIS
    public static double min_command = 0.05;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor in inches
    double goalHeightInches = 29.5; //29.5" from floor to apriltag

    // Desired distance from tag (adjustable)
    public static double desiredDistanceInches = 24.0;

    private Limelight3A limelight;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double y  = -gamepad1.left_stick_y;   // forward/back
        double x  = gamepad1.left_stick_x * 1.1; // strafe
        double rx = gamepad1.right_stick_x;   // rotation

        telemetry.addData("Name", "%s", status.getName());
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

            telemetry.addData("Distance (in)", distanceFromLimelightToGoalInches);

            // Heading correction when right trigger held
            if (gamepad1.right_trigger > 0.3) {
                double heading_error = -tx;
                double steering_adjust = 0.0;

                if (Math.abs(heading_error) > 1.0) {
                    if (heading_error < 0) {
                        steering_adjust = Kp * heading_error + min_command;
                    } else {
                        steering_adjust = Kp * heading_error - min_command;
                    }
                }

                // Override joystick turn with auto-align correction
                rx = steering_adjust;
                telemetry.addData("Heading-Lock", "ACTIVE");

                // Drive forward/back to maintain desired distance
                double distanceError = distanceFromLimelightToGoalInches - desiredDistanceInches;
                y = -Dp * distanceError; // keep Dp negative so it will drive in the right direction

                telemetry.addData("Distance Error", distanceError);
            } else {
                telemetry.addData("Heading-Lock", "OFF");
            }

        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        // Mecanum drive calculation
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        telemetry.update();
    }
}
