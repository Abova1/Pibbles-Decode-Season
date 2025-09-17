package org.firstinspires.ftc.teamcode.util;

//GOBUILDA ESTIMATES ARE BASED ON DUAL MODE SERVOS

/* SPEED AND TORQUE AT 6V (SERVO HUB)
 * AXON -
 * MAX 0.116 | 34
 * MINI 0.090 | 25
 * MICRO 0.075 | 7.8
 *
 *
 * SWYFT -
 * TORQUE 0.112 | 33.5
 * BALANCE 0.092 | 27.3
 * SPEED 0.062 | 19
 *
 * GOBUILDA -
 * TORQUE 0.20 | 21.6
 * SPEED 0.090 | 9.3
 * SUPERSPEED  0.043 | 4.7
 * */

/* SPEED AND TORQUE AT 4.8V (CONTROL/EXPANSION HUB)
 * AXON -
 * MAX 0.140 | 28
 * MINI 0.110 | 20
 * MICRO 0.085 | 6.5
 *
 *
 * SWYFT - Doesn't say anything about Voltage that I could find so I'm using the same values
 * TORQUE 0.112 | 33.5
 * BALANCE 0.092 | 27.3
 * SPEED 0.062 | 19
 *
 * GOBUILDA -
 * TORQUE 0.25 | 17.2
 * SPEED 0.11  | 7.9
 * SUPERSPEED  0.055 | 4.0
 * */

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Command.*;

public class Globals {

    ElapsedTime GlobalTimer = new ElapsedTime();

    public enum Alliance {
        RED,
        BLUE
    }

    public enum CHUB_SERVO_TYPES {
        AXON_MAX(60.0 / 0.140, 28),
        AXON_MINI(60.0 / 0.110, 20),
        AXON_MICRO(60.0 /  0.085, 6.5),
        SWYFT_TORQUE(60.0 / 0.112, 33.5 ),
        GOBUILDA_TORQUE(60.0 / 0.25, 17.2),
        SWYFT_BALANCE(60.0 / 0.092, 27.3),
        SWYFT_SPEED(60.0 / 0.062, 19),
        GOBUILDA_SPEED(60.0 / 0.11, 7.9),
        GOBUILDA_SUPERSPEED(60.0 / 0.055 ,  4.0);

        public final double speed;
        public final double stallTorque;

        CHUB_SERVO_TYPES(double speed, double torque ){
            this.speed = speed;
            this.stallTorque = torque;
        }

    }

    public enum SERVO_TYPES {
        AXON_MAX(60.0 / 0.115, 34.0),
        AXON_MINI(60.0 / 0.90, 25),
        AXON_MICRO(60.0 / 0.075, 7.8),
        SWYFT_TORQUE(60.0 / 0.112, 33.5 ),
        GOBUILDA_TORQUE(60.0 / 0.20, 21.6),
        SWYFT_BALANCE(60.0 / 0.092, 27.3),
        SWYFT_SPEED(60.0 / 0.062, 19),
        GOBUILDA_SPEED(60.0 /  0.090, 9.3),
        GOBUILDA_SUPERSPEED(60.0 / 0.043, 4.7);

        public final double speed;
        public final double stallTorque;

        SERVO_TYPES(double speed, double torque ){
            this.speed = speed;
            this.stallTorque = torque;
        }

    }

    public enum SERVO_TYPES_ANGLES {
        //THESE ARE ALL THE MAX ANGLES THAT YOU CAN PROGRAM IT TO

        AXON(355),
        SWYFT(320),
        GOBUILDA(300);

        double Angle;

        SERVO_TYPES_ANGLES(double Angle){
            this.Angle = Angle;
        }

    }

    public static double getAngle(SERVO_TYPES_ANGLES servo, double position){
        return position * servo.Angle;
    }

    public static long SHUBArmEstimates(SERVO_TYPES servo, int numServos, double armMass /*In KG*/, double armLength /*In CM*/, double angle){

        double GRAVITY = 9.81;

        double base = angle / servo.speed;

        double loadTorque = (armMass * GRAVITY * armLength ) / GRAVITY;

        double totalTorque = servo.stallTorque * numServos;

        double load = 1 + (loadTorque / totalTorque);

        double estimatedTime = base * load;

        return (long) (estimatedTime * 1000);
    }

    public static long CHUBArmEstimates(CHUB_SERVO_TYPES servo, int numServos, double armMass /*In KG*/, double armLength /*In CM*/, double angle){

        double GRAVITY = 9.81;

        double base = angle / servo.speed;

        double loadTorque = (armMass * GRAVITY * armLength ) / GRAVITY;

        double totalTorque = servo.stallTorque * numServos;

        double load = 1 + (loadTorque / totalTorque);

        double estimatedTime = base * load;

        return (long) (estimatedTime * 1000);
    }

    public static double clamp(double value, double max, double min){
        if(value > max){
            return max;
        }
        else if(value < min){
            return min;
        }
        return value;
    }

    //Only Useful for anything other than Parallel Commands
    public Command waitFor(double milliseconds){
        return new Command() {
            boolean finished = false;
            @Override
            public void init() {
                GlobalTimer.reset();
            }
            @Override
            public void execute() {
                if(GlobalTimer.milliseconds() >= milliseconds){
                    finished = true;
                }
            }
            @Override
            public boolean isFinished() {
                return finished;
            }
            @Override
            public void end(boolean cancelled) {}
        };
    }


}