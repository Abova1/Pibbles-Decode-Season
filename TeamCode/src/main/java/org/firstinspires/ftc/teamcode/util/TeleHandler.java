package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Turret.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Commands.*;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooters;
import org.firstinspires.ftc.teamcode.util.Command.*;


public class TeleHandler {

    private RobotState state = RobotState.REGULAR;
    private Shooters shooter;
    private Turret turret;
    private Controller Driver, Operator;
    private CommandScheduler scheduler;
    private Command maxVeloCommand;
    private Command minVeloCommand;
    private Object[] subsystems;

    public TeleHandler(Controller driver, CommandScheduler scheduler, Object... subsystems){

        this.Driver = driver;
        this.scheduler = scheduler;
        this.subsystems = subsystems;

        for (Object subsystem : subsystems) {

            if (subsystem instanceof Shooters) {
                this.shooter = (Shooters) subsystem;
            }

            if (subsystem instanceof Turret) {
                this.turret = (Turret) subsystem;
            }

        }

        maxVeloCommand = new maxVelocityCommand();
        minVeloCommand = new minVelocityCommand();

    }

    public RobotState getState(){
        return state;
    }

    public void TeleOp (){

        if(shooter != null){
            shooter.run();
        }

        if(turret != null){
            turret.run();
        }

        switch (state){

            case REGULAR:


            break;
        }

    }

}