package org.firstinspires.ftc.teamcode.subsystems.shooter.Commands;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooters;
import org.firstinspires.ftc.teamcode.util.Command.Command;

public class maxVelocityCommand implements Command {

    private final double MAX_Velocity = 3100;
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

        if (Shooters.target != MAX_Velocity) {
            Shooters.target = MAX_Velocity;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean cancelled) {

        if (cancelled) {
            Shooters.target = MIN_Velocity;
        }

    }

}
