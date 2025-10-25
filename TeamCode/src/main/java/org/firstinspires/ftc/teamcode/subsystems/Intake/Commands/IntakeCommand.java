package org.firstinspires.ftc.teamcode.subsystems.Intake.Commands;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.util.Command.Command;

public class IntakeCommand implements Command {

    private final double MAX_Velocity = 10000;// Undefined
    private final double MIN_Velocity = 0;

    private String name = "Intake Command";


    public IntakeCommand(){}

    @NonNull
    @Override
    public String toString(){
        return "[name=" + name + "]";
    }


    @Override
    public void init() {} //When the time comes we'll need this

    @Override
    public void execute() {

        Intake.setIntakeTarget(MAX_Velocity);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean cancelled) {

        if (cancelled) {
            Intake.setIntakeTarget(MIN_Velocity);
        }

    }

}
