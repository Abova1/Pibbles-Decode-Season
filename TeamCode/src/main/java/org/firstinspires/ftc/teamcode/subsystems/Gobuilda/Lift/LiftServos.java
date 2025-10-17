package org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Lift;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Command.Command;
import org.firstinspires.ftc.teamcode.util.Globals;

public class LiftServos {

    private Servo topServo, middleServo, bottomServo;

    private ElapsedTime servoTimer = new ElapsedTime();



    public LiftServos(HardwareMap hardwareMap){

        topServo = hardwareMap.get(Servo.class, "Top");
        middleServo = hardwareMap.get(Servo.class, "Middle");
        bottomServo = hardwareMap.get(Servo.class, "Bottom");

        middleServo.setDirection(Servo.Direction.REVERSE);
        reset();

    }

    public void reset (){
        topServo.setPosition(Values.TopNeutral);
        middleServo.setPosition(Values.MiddleNeutral);
        bottomServo.setPosition(Values.BottomNeutral);
    }

    public double getTopServoPos(){
        return topServo.getPosition();
    }
    public double getMidServoPos(){
        return middleServo.getPosition();
    }
    public double getBottomServoPos(){
        return bottomServo.getPosition();
    }

    public double getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES angles, double pos, double generalAngleWanted){
        return Math.abs(Globals.getAngle(angles, pos) - generalAngleWanted);
    }

    public Command TopLift(){
        return new Command() {
            long timeNeeded;
            @Override
            public void init() {

                servoTimer.reset();
                timeNeeded = Globals.SHUBArmEstimates(Globals.SERVO_TYPES.GOBUILDA_SPEED, 1, 0 ,0, getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES.GOBUILDA, getTopServoPos(), Values.TopLiftAngle));

            }

            @Override
            public void execute() {

                topServo.setPosition(Values.TopLift);

            }

            @Override
            public boolean isFinished() {
                return servoTimer.milliseconds() >= timeNeeded;
            }

            @Override
            public void end(boolean cancelled) {
                if(cancelled){
                    topServo.setPosition(Values.TopNeutral);
                }
            }
        };
    }

    public Command MiddleLift(){
        return new Command() {
            long timeNeeded;
            @Override
            public void init() {

                servoTimer.reset();
                timeNeeded = Globals.SHUBArmEstimates(Globals.SERVO_TYPES.GOBUILDA_SPEED, 1, 0 ,0, getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES.GOBUILDA, getMidServoPos(), Values.MiddleLiftAngle));

            }

            @Override
            public void execute() {

                middleServo.setPosition(Values.MiddleLift);

            }

            @Override
            public boolean isFinished() {
                return servoTimer.milliseconds() >= timeNeeded;
            }

            @Override
            public void end(boolean cancelled) {
                if(cancelled){
                    middleServo.setPosition(Values.MiddleNeutral);
                }
            }
        };
    }

    public Command BottomLift(){
        return new Command() {
            long timeNeeded;
            @Override
            public void init() {

                servoTimer.reset();
                timeNeeded = Globals.SHUBArmEstimates(Globals.SERVO_TYPES.GOBUILDA_SPEED, 1, 0 ,0, getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES.GOBUILDA, getBottomServoPos(), Values.BottomLiftAngle));

            }

            @Override
            public void execute() {

                bottomServo.setPosition(Values.BottomLift);

            }

            @Override
            public boolean isFinished() {
                return servoTimer.milliseconds() >= timeNeeded;
            }

            @Override
            public void end(boolean cancelled) {
                if(cancelled){
                    bottomServo.setPosition(Values.BottomNeutral);
                }
            }
        };
    }

}
