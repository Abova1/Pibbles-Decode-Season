package org.firstinspires.ftc.teamcode.subsystems.Chamber;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Command.Command;
import org.firstinspires.ftc.teamcode.util.Globals;

public class Chamber {

    private Servo chamber, holder;
    private ElapsedTime chamberTimer = new ElapsedTime();
    private ElapsedTime holderTimer = new ElapsedTime();


    public Chamber(HardwareMap hardwareMap) {

        holder = hardwareMap.get(Servo.class, "holder");
        chamber = hardwareMap.get(Servo.class, "chamber");

        reset();
    }

    public void reset (){
        chamber.setPosition(Constants.ChamberInside);
        holder.setPosition(Constants.HolderDown);
    }

    public double getChamberPos(){
        return chamber.getPosition();
    }
    public double getHolderPos(){
        return holder.getPosition();
    }

    public double getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES angles, double pos, double generalAngleWanted){
        return Math.abs(Globals.getAngle(angles, pos) - generalAngleWanted);
    }

    public double getChamberInAngle(){
        return Globals.getAngle(Globals.SERVO_TYPES_ANGLES.AXON, Constants.ChamberInside);
    }

    public double getChamberOutAngle(){
        return Globals.getAngle(Globals.SERVO_TYPES_ANGLES.AXON, Constants.ChamberOut);
    }

    public double getHolderUpAngle(){
        return Globals.getAngle(Globals.SERVO_TYPES_ANGLES.AXON, Constants.HolderUp);
    }
    public double getHolderDownAngle(){
        return Globals.getAngle(Globals.SERVO_TYPES_ANGLES.AXON, Constants.HolderDown);
    }


    public Command chamberIn(){
        return new Command() {

            double angleDeg = getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES.AXON, getChamberPos(), getChamberInAngle());
            long timeNeeded;
            @Override
            public void init() {
                chamberTimer.reset();
                timeNeeded = Globals.pulleyTimeEstimate(Globals.CHUB_SERVO_TYPES.AXON_MAX, angleDeg, 0 ,0);
            }

            @Override
            public void execute() {
                chamber.setPosition(Constants.ChamberInside);
            }

            @Override
            public boolean isFinished() {
                return chamberTimer.milliseconds() > timeNeeded;
            }

            @Override
            public void end(boolean cancelled) {
            }
        };
    }


    public Command chamberOut(){
        return new Command() {

            double angleDeg = getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES.AXON, getChamberPos(), getChamberOutAngle());
            long timeNeeded;

            @Override
            public void init() {
                chamberTimer.reset();
                timeNeeded = Globals.pulleyTimeEstimate(Globals.CHUB_SERVO_TYPES.AXON_MAX, angleDeg, 0 ,0);
            }

            @Override
            public void execute() {
                chamber.setPosition(Constants.ChamberOut);
            }

            @Override
            public boolean isFinished() {
                return chamberTimer.milliseconds() > timeNeeded;
            }

            @Override
            public void end(boolean cancelled) {
            }
        };
    }

    public Command Hold(){
        return new Command() {

            long timeNeeded;
            @Override
            public void init() {
                holderTimer.reset();
                timeNeeded = Globals.ArmEstimates(Globals.CHUB_SERVO_TYPES.AXON_MICRO, 1, 0,0, getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES.AXON, getHolderPos(), getHolderUpAngle()));;
            }

            @Override
            public void execute() {
                holder.setPosition(Constants.HolderUp);
            }

            @Override
            public boolean isFinished() {
                return holderTimer.milliseconds() > timeNeeded;
            }

            @Override
            public void end(boolean cancelled) {
            }
        };
    }

    public Command Release(){
        return new Command() {

            long timeNeeded;
            @Override
            public void init() {
                holderTimer.reset();
                timeNeeded = Globals.ArmEstimates(Globals.CHUB_SERVO_TYPES.AXON_MICRO, 1, 0,0, getTimeNeededAngle(Globals.SERVO_TYPES_ANGLES.AXON, getHolderPos(), getHolderDownAngle()));;
            }

            @Override
            public void execute() {
                holder.setPosition(Constants.HolderDown);
            }

            @Override
            public boolean isFinished() {
                return holderTimer.milliseconds() > timeNeeded;
            }

            @Override
            public void end(boolean cancelled) {
            }
        };
    }



}
