package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooters;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class TeleHandler {

    private RobotState state = RobotState.REGULAR;
    private Shooters shooter;
    private Controller Driver, Operator;


    private Globals Globals;
    private CommandScheduler scheduler;


    public TeleHandler(Controller driver, CommandScheduler scheduler, Shooters shooter){

        this.Driver = driver;
        this.scheduler = scheduler;
        this.shooter = shooter;

    }

    public RobotState getState(){
        return state;
    }

    public void TeleOp (){


        switch (state){

            case REGULAR:


            break;

            case INVERTED:


                break;

        }

    }

}