package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Commands.*;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooters;
import org.firstinspires.ftc.teamcode.util.Command.*;


public class TeleHandler {

    private RobotState state = RobotState.REGULAR;
    private Shooters shooter;
    private Controller Driver, Operator;
    private CommandScheduler scheduler;

    private Command maxVeloCommand;
    private Command minVeloCommand;



    public TeleHandler(Controller driver, CommandScheduler scheduler, Shooters shooter){

        this.Driver = driver;
        this.scheduler = scheduler;
        this.shooter = shooter;

        maxVeloCommand = new maxVelocityCommand();
        minVeloCommand = new minVelocityCommand();

    }

    public RobotState getState(){
        return state;
    }

    public void TeleOp (){

        shooter.run();

        switch (state){

            case REGULAR:


            break;


        }

    }

}