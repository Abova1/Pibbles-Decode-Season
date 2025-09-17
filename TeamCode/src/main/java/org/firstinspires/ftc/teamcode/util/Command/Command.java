package org.firstinspires.ftc.teamcode.util.Command;

/*
I didn't want to use FTC libs Command class
 */

public interface Command {
    void init();
    void execute();
    boolean isFinished();
    void end(boolean cancelled);
}