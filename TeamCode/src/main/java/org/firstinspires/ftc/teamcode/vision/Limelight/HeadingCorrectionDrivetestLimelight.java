package org.firstinspires.ftc.teamcode.vision.Limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
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
@TeleOp(name="Configurable AprilTag Limelight", group="Vision")
public class HeadingCorrectionDrivetestLimelight extends OpMode {

    // TODO: 9/20/2025 this seems wrong for some reason check out field centric code to see how to manipulate heading better

    /*
     pipeline 1 is red goal
    pipeline 2 is blue goal
    pipeline 3 is motif
     */
    public static double Kp = -0.1;

    //theres also a PContoller for ftclib
    public PController controller;



    public static double min_command = 0.05;

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

        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());


        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid() && gamepad1.right_trigger > 0.3) {
            double tx = result.getTx(); // target offset

            double heading_error = -tx;
            double steering_adjust = 0.0;

            /*

            How you would implement this is

            double input = headingError + min_command;
            1 + 0.05 = 1.05;
            1 + -0.05

            if (Math.abs(heading_error) > 1.0) {

              steering adjust = controller.calculate(headingError, targetHeading);

            }

            or something in that sense


             */

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
            telemetry.addData("Target X", tx);
        } else {
            telemetry.addData("Heading-Lock", "OFF");
        }

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
