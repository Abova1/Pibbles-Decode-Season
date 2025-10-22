package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PIDWrapper;

public class Turret {

    private DcMotorEx turret;
    private Servo hood;
    private AnalogInput Encoder;
    private PIDWrapper TurretController = new PIDWrapper(new PIDController(Constants.Tp, Constants.Ti, Constants.Td));
    private Sensors sensors;
    private double targetHeading;
    private Limelight3A limelight;
    private double heading;
    private double distanceFromLimelightToGoalInches;
    private double ServoPosCalculation;
    private boolean leftBoolean;
    private boolean rightBoolean;
    private double leftDouble;
    private double rightDouble;



    public Turret(HardwareMap hardwareMap){

        turret = hardwareMap.get(DcMotorEx.class, "motor0");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(150);


        Encoder = hardwareMap.get(AnalogInput.class, "Turret Encoder");

        sensors = new Sensors(hardwareMap);
        sensors.initIMU();
        sensors.resetIMUYaw();

        reset();

        heading = getMEHeading();

    }

    public void setTurretPower(double power){
        turret.setPower(power);
    }

    public double getTurretPower(){
        return turret.getPower();
    }
    public double getTurretAMPS(){
        return turret.getCurrent(CurrentUnit.AMPS);
    }

    public double getTurretPos(){
        return turret.getCurrentPosition();
    }
    public double getHoodPos(){
        return hood.getPosition();
    }
    public void setHoodPos(double pos){
        hood.setPosition(pos);
    }


    public void reset(){

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //ANALOG ENCODER VALUES
    public double getAnalogHeading() {

        double voltage = Encoder.getVoltage();

        double degrees = (voltage / 3.3) * 360;

        return (((degrees % 360) + 360) % 360);
    }

    //ME is MOTOR ENCODER
    public double getMEHeading() {

        double TPR;

        if(Constants.GearRatio != 0){
            TPR = Constants.TPR * Constants.GearRatio;
        } else {
            TPR = Constants.TPR;
        }

        double ticks = getTurretPos();
        double degrees = (ticks / TPR) * 360;

        return degrees % 360;

    }


    public void setTargetHeading (double degrees){
        targetHeading = degrees;
    }
    public double getServoPosCalculation() {
        return ServoPosCalculation;
    }

    public void setManualButtons(double left, double right){
        leftDouble = left;
        rightDouble = right;

    }

    public void setManualButtons(boolean left, boolean right){
        leftBoolean = left;
        rightBoolean = right;
    }

    public void runAutoTracking(){

        LLResult result = limelight.getLatestResult();

        TurretController.setPID(Constants.Tp, Constants.Ti, Constants.Td);
        TurretController.setInverse(true);

        double headingError = targetHeading - heading;

        if (heading > Constants.upperThreshold && headingError > 0) {
            // About to go past upper limit, unwrap clockwise
            targetHeading -= 360;
        } else if (heading < Constants.lowerThreshold && headingError < 0) {
            // About to go past lower limit, unwrap counter-clockwise
            targetHeading += 360;
        }

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            targetHeading -= Constants.kTx * tx;

            double targetOffsetAngle_Vertical = ty;
            double angleToGoalDegrees = Constants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            distanceFromLimelightToGoalInches = (Constants.goalHeightInches - Constants.limelightLensHeightInches) / Math.tan(angleToGoalRadians);


        } else {

            if(rightBoolean){
                targetHeading -= 1.5;
            }else if(leftBoolean){
                targetHeading += 1.5;
            }
            /*================different button/trigger choices=====================*/
            else if(rightDouble > 0.5){
                targetHeading -= 1.5;
            }else if(leftDouble > 0.5){
                targetHeading += 1.5;
            }


        }

        //0.1 is down 0.55 is up
        ServoPosCalculation = ((-0.00391524) * (distanceFromLimelightToGoalInches)) + 0.696695;
        ServoPosCalculation = Globals.clamp(ServoPosCalculation, 0.55, 0.1);

        hood.setPosition(ServoPosCalculation);
        TurretController.PositionRun(heading, targetHeading, turret);

    }

}
