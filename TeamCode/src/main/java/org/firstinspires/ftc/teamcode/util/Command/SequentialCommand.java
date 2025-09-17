package org.firstinspires.ftc.teamcode.util.Command;

import java.util.*;

public class SequentialCommand implements Command {

    private final Queue<Command> commands;
    private Command current = null;

    public SequentialCommand(Command... cmds) {
        this.commands = new LinkedList<>(Arrays.asList(cmds));
    }

    @Override
    public void init() {
        if (!commands.isEmpty()) {
            current = commands.poll();
            current.init();
        }
    }

    @Override
    public void execute() {
        if (current == null) return;

        current.execute();

        if (current.isFinished()) {
            current.end(false);

            if (!commands.isEmpty()) {
                current = commands.poll();
                current.init();
            } else {
                current = null;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return current == null;
    }

    @Override
    public void end(boolean cancelled) {

        if(cancelled){
            for (Command command : new ArrayList<>(commands)){
                command.end(true);
            }
        }

        else {
            if (current != null) {
                current.end(false);
            }
        }


    }
}