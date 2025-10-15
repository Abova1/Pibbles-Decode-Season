package org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Intake;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.util.Command.Command;

public class OuttakeCommand implements Command {

    private String name = "Outtake Command";

    public OuttakeCommand(){}

    @NonNull
    @Override
    public String toString(){
        return "[name=" + name + "]";
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {

        if (GobuildaIntake.target != Values.NEGATIVE_MAX) {
            GobuildaIntake.target = Values.NEGATIVE_MAX;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean cancelled) {
        if (cancelled) {
            GobuildaIntake.target = Values.ZERO;
        }
    }
}
