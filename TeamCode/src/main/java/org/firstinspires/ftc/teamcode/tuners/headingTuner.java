package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.DT;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.DataStorage;
import org.firstinspires.ftc.teamcode.util.Globals;


@Config
@TeleOp(name="Heading Tuner", group="tuners")
public class headingTuner extends OpMode {

    //myles poopy head2

    private DcMotorEx motor;
    private PIDController controller;
    private Limelight3A limelight;
    private DT drivetrain;

    private Sensors sensors;

    public static double p = 0.01875, i = 0.005, d = 0.001475;

    public static double ticksPerRev = 407.785714286;
    public static double targetHeading = 0;
    public static double MAX = 1;
    public static double MIN = -1;
    private double previousHeading;
    private double heading;
    private String direction = " ";
    private String positive = "positive";
    private String negative = "negative";

    public static double kTx = 0.07; // how aggressively tx affects targetHeading



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDController(p, i, d);
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain = new DT(hardwareMap);

        sensors = new Sensors(hardwareMap);
        sensors.initIMU();

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        previousHeading = DataStorage.loadHeading();

    }

    @Override
    public void start(){
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(150);
    }


    @Override
    public void loop() {

        drivetrain.RCDrive(-(gamepad1.left_stick_y), gamepad1.left_stick_x, gamepad1.right_stick_x);

        LLStatus status = limelight.getStatus();

        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();

        controller.setPID(p, i, d);

        double ticks = motor.getCurrentPosition();
        double degrees = (ticks / ticksPerRev) * 360;

        heading = ((degrees % 360));


        if(heading >= 134){
            targetHeading = -215;
        }
        else if(heading <= -224){
            targetHeading = 130;
        }

        double power = controller.calculate(heading, targetHeading);

//        power = Globals.clamp(power, MAX, MIN);

        motor.setPower(-power);

        double headingError = targetHeading - heading;

        if(power < 0){
            direction = negative;
        } else if(power > 0){
            direction = positive;
        } else {
            direction = "No Direction";
        }

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);

            targetHeading -= kTx * tx;


        } else {

        }


        telemetry.addData("Calculation", power);
        telemetry.addData("Ticks", ticks);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Heading", heading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("Direction", direction);
        telemetry.addData("Previous Heading", previousHeading);
        telemetry.update();

    }

    @Override
    public void stop(){

        DataStorage.saveHeading(heading);

    }

}
