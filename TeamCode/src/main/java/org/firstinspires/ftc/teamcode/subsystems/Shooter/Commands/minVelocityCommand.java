package org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.Command.Command;

public class minVelocityCommand implements Command {

    private final double MAX_Velocity = 2000;
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

        if (Shooter.target != MAX_Velocity) {
            Shooter.setShooterTarget(MAX_Velocity);
        }

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
