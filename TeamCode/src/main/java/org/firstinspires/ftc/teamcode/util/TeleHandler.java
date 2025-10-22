package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Shooter.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands.maxVelocityCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands.minVelocityCommand;
import org.firstinspires.ftc.teamcode.util.Command.*;

public class TeleHandler {

    private RobotState state = RobotState.REGULAR;
    private Controller Driver, Operator;
    private Shooter shooter;
    private Object[] subsystems;
    private CommandScheduler scheduler;

    private Command maxShooterVelo;
    private Command minShooterVelo;

    public TeleHandler(Controller driver, CommandScheduler scheduler, Object... subsystems){

        this.Driver = driver;
        this.scheduler = scheduler;
        this.subsystems = subsystems;

        for (Object subsystem : subsystems) {

            if (subsystem instanceof Shooter) {
                this.shooter = (Shooter) subsystem;
            }

        }

        maxShooterVelo = new maxVelocityCommand();
        minShooterVelo = new minVelocityCommand();

    }

    public RobotState getState(){
        return state;
    }

    public void TeleOp (){

        if(shooter != null){
            shooter.run();
        }

        if(scheduler != null){
            scheduler.run();
        }

        switch (state){

            case REGULAR:

                Driver.buttonPressed(Driver::a, ()-> maxShooterVelo);
                Driver.buttonPressed(Driver::b, ()-> minShooterVelo);

            break;
        }

    }

}