package org.firstinspires.ftc.teamcode.util.Command;

import java.util.Arrays;
import java.util.List;
import java.util.Queue;

public class ParallelCommand implements Command {

    private List<Command> commands;

    public ParallelCommand(Command... commands){

        this.commands = Arrays.asList(commands);

    }

    public List<Command> getCommands() {
        return commands;
    }

    @Override
    public void init() {

        for(Command command: commands){
            command.init();
        }

    }

    @Override
    public void execute() {

        for(Command command: commands ){
            command.execute();
        }

    }

    @Override
    public boolean isFinished() {

        for (Command command : commands){

          if(!command.isFinished()){
              return false;
          }

        }

        return true;
    }

    @Override
    public void end(boolean cancelled) {

        for (Command command : commands) {

            if (command != null) {
                command.end(cancelled);
            }

        }

    }

}