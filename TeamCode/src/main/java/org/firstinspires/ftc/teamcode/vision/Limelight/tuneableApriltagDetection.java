package org.firstinspires.ftc.teamcode.vision.Limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.PIDWrapper;


@Configurable
@Config
@TeleOp(name="Configurable AprilTag Limelight", group="Vision")
public class tuneableApriltagDetection extends OpMode {

    /*
     pipeline 1 is red goal
    pipeline 2 is blue goal
    pipeline 3 is motif
     */

     /* Myles's P and D
    double kP = 0.01;
    double kD = 0.002;
     */

    private Limelight3A limelight;
    private DcMotorEx LLmotor;

    public static double
            kP = 0 /* proportional gain (tune this) */,
            kD = 0
    ;

    public double targetTx = 0;
    public static double txTolerance = 1.5;
    public static double powerThreshold = 0.02; // don’t bother running motor if |power| < 0.02
    public static double maxPower = 0.3;
    private PIDWrapper controller = new PIDWrapper(new PDController(kP, kD));
    public static String Color = "red";


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LLmotor = hardwareMap.get(DcMotorEx.class, "LLmotor");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        LLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Personally I think this is better but we'll see
        LLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        controller.setPD(kP, kD);

        if(Color.equalsIgnoreCase("red")){
            limelight.pipelineSwitch(1);
        }
        else if(Color.equalsIgnoreCase("blue")){
            limelight.pipelineSwitch(2);
        }

        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData("Motor power", LLmotor.getPower()); // same thing if you used the variable

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);

            controller.TurretRun(tx, targetTx, maxPower, -maxPower, powerThreshold, txTolerance, LLmotor);

        } else {
            LLmotor.setPower(0);
            telemetry.addData("Limelight", "No Targets");
        }
    }

}
