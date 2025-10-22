package org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.Command.Command;

public class maxVelocityCommand implements Command {

    private final double MAX_Velocity = 2580;
    private final double MIN_Velocity = 0;

    private String name = "MaxVeloCommand";


    public maxVelocityCommand(){}

    @NonNull
    @Override
    public String toString(){
        return "[name=" + name + "]";
    }


    @Override
    public void init() {} //When the time comes we'll need this

    @Override
    public void execute() {

        Shooter.setShooterTarget(MAX_Velocity);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean cancelled) {

        if (cancelled) {
            Shooter.setShooterTarget(MIN_Velocity);
        }

    }

}
