package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
@TeleOp
public class ServoTest extends OpMode {

    Servo chamber, hood, holder;

    public static double pos;
    public static boolean Reverse = false;
    public static String ServoName = "";
    public String hoodName = "hood";
    public String holderName = "holder";
    public String chamberName = "chamber";


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chamber = hardwareMap.get(Servo.class, "chamber");
        holder = hardwareMap.get(Servo.class, "holder");
        hood = hardwareMap.get(Servo.class, "hood");


    }

    @Override
    public void loop() {


        if(ServoName.equalsIgnoreCase(hoodName)){
            hood.setPosition(pos);
        }else if(ServoName.equalsIgnoreCase(holderName)){
            holder.setPosition(pos);
        }else if(ServoName.equalsIgnoreCase(chamberName)){
            chamber.setPosition(pos);
        }

        if(Reverse){
            hood.setDirection(Servo.Direction.REVERSE);
            holder.setDirection(Servo.Direction.REVERSE);
            chamber.setDirection(Servo.Direction.REVERSE);
        } else {
            hood.setDirection(Servo.Direction.REVERSE);
            holder.setDirection(Servo.Direction.REVERSE);
            chamber.setDirection(Servo.Direction.REVERSE);
        }

        telemetry.addData("ServoPos", chamber.getPosition());
        telemetry.addData("Direction", chamber.getDirection());
        telemetry.update();

    }
}
