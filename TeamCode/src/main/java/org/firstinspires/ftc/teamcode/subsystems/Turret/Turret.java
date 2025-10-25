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

    private PIDController controller;
    private Sensors sensors;
    private double targetHeading;
    private Limelight3A limelight;
    private double turretHeading;
    private double distanceFromLimelightToGoalInches;
    private double distance;
    private double ServoPosCalculation;
    private boolean leftBoolean;
    private boolean rightBoolean;
    private double leftDouble;
    private double rightDouble;
    public double turretAngle;





    public Turret(HardwareMap hardwareMap){

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");

        controller = new PIDController(Constants.Tp, Constants.Ti, Constants.Td);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(150);


//        Encoder = hardwareMap.get(AnalogInput.class, "Turret Encoder");

        sensors = new Sensors(hardwareMap);
        sensors.initIMU();
        sensors.resetIMUYaw();

        reset();

        turretHeading = getMEHeading();

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

    public double getPDistance(){
        return distance;
    }


    public void reset(){

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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



    public double getTurretAngle(){
        return turretAngle;
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


    public void runHood(){

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double targetOffsetAngle_Vertical = result.getTy();
            double angleToGoalDegrees = Constants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            distanceFromLimelightToGoalInches = (Constants.goalHeightInches - Constants.limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        }

        ServoPosCalculation = ((-0.00391524) * (distanceFromLimelightToGoalInches)) + 0.696695;
        ServoPosCalculation = Globals.clamp(ServoPosCalculation, Constants.hoodUp, Constants.hoodDown);

    }

    public void alignTurret(double x, double y, double heading, boolean blue) {
        double RobotHeadingDeg = Math.toDegrees(heading);
        double goalX = blue ? Constants.blueGoalX : Constants.redGoalX;
        double goalY = blue ? Constants.blueGoalY : Constants.redGoalY;

        x = Constants.turretOffsetX + x;
        y = Constants.turretOffsetY + y;

        double angleToGoal = Math.toDegrees(Math.atan2(goalX - x, goalY - y));
        turretAngle = angleToGoal + RobotHeadingDeg - 90;



        // you can either clamp, or try this:
        /*
        if (turretAngle > Constants.upperThreshold) {
        turretAngle -= 360;
    } else if (turretAngle < Constants.lowerThreshold) {
        turretAngle += 360;
    }
         */

        // you have to reassign the value
        turretAngle = Globals.clamp(turretAngle, Constants.upperThreshold, Constants.lowerThreshold);


        distance = Math.hypot(goalX-x, goalY-y);
        ServoPosCalculation = ((-0.00391524) * (distance)) + 0.696695; //REDO THIS EQUATION SINCE ITS USING OLD LL EQUATION!!!!!!!
        ServoPosCalculation = Globals.clamp(ServoPosCalculation, Constants.hoodUp, Constants.hoodDown);

        controller.setPID(Constants.Tp, Constants.Ti, Constants.Td);

        int targetTicks = (int) (Constants.tSlope * turretAngle);
        double power = controller.calculate(getTurretPos(), targetTicks);
        turret.setPower(-power);


    }


    public void runAutoTrackingLL(){

        LLResult result = limelight.getLatestResult();

        TurretController.setPID(Constants.Tp, Constants.Ti, Constants.Td);
        TurretController.setInverse(true);

        double headingError = targetHeading - turretHeading;

        if (turretHeading > Constants.upperThreshold && headingError > 0) {
            // About to go past upper limit, unwrap clockwise
            targetHeading -= 360;
        } else if (turretHeading < Constants.lowerThreshold && headingError < 0) {
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
        TurretController.PositionRun(turretHeading, targetHeading, turret);


    }


}
