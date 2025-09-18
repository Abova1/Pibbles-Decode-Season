package org.firstinspires.ftc.teamcode.subsystems.shooter.Commands;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooters;
import org.firstinspires.ftc.teamcode.util.Command.Command;

public class minVelocityCommand implements Command {

    private final double MAX_Velocity = 1000;
    private final double MIN_Velocity = 0;
    private String name = "Min Velocity Command";

    public minVelocityCommand(){}

    @NonNull
    @Override
    public String toString(){
        return "[name=" + name + "]";
    }

    @Override
    public void init() {}

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
