package org.firstinspires.ftc.teamcode.util.Command;

import java.util.Arrays;
import java.util.List;

public class ParallelCommand implements Command {

    private List<Command> commands;

    public ParallelCommand(Command ... commands){

        this.commands = Arrays.asList(commands);

    }


    @Override
    public void init() {

        for(Command command: commands ){
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

        if(cancelled){

            for (Command command : commands){
                command.end(true);
            }

        }
        else {

            for (Command command : commands){
                command.end(false);
            }

        }
    }

}