package org.firstinspires.ftc.teamcode.subsystems.Chamber;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Guiderails {

    private Servo Guiderails;


    public Guiderails(HardwareMap hardwareMap) {

        Guiderails = hardwareMap.get(Servo.class, "GuideRail");


    }

}
