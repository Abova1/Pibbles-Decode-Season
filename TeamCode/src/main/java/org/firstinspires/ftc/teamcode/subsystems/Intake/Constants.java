package org.firstinspires.ftc.teamcode.subsystems.Intake;

class Constants {

    public final static double Ip = 0.001, Ii = 0, Id = 0, If = 0;//PID tune
    public final static double VeloErrorThresh = 50;

    //KEEP THIS ABOVE 0.0009
    public static double alpha = 0.01;

    //We'll set these later
    public static double MAX_VELOCITY = 10000;
    public static double MIN_VELOCITY = -10000;

}
