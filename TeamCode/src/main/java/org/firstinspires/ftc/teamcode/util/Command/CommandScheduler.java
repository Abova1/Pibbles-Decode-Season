package org.firstinspires.ftc.teamcode.util.Command;

import java.util.*;

public class CommandScheduler {

    private final List<Command> activeCommands = new ArrayList<>();

    public void schedule(Command command){

        if(!activeCommands.contains(command)){
            command.init();
            activeCommands.add(command);
        }

    }

    public void run(){

        Iterator<Command> iterator = activeCommands.iterator();

        while (iterator.hasNext()){

            Command command = iterator.next();

            command.execute();

            if(command.isFinished()){

                command.end(false);
                iterator.remove();

            }

        }

    }

    public boolean isScheduled(Command command){
        return activeCommands.contains(command);
    }

    public void clear(){
        activeCommands.clear();
    }

    public String currentCommandScheduled() {
        if (activeCommands.isEmpty()) {
            return "No command scheduled.";
        }
        return activeCommands.get(0).toString();
    }

    public void cancel(Command command) {

        if(activeCommands.contains(command)){
            command.end(true);
            activeCommands.remove(command);
        }

        if (command instanceof SequentialCommand) {
            SequentialCommand sequentialCommand = (SequentialCommand) command;
            for (Command cmd : sequentialCommand.getCommands()) {
                cancel(cmd);
            }
        }


        if (command instanceof ParallelCommand) {
            ParallelCommand parallelCommand = (ParallelCommand) command;
            for (Command cmd : parallelCommand.getCommands()) {
                cancel(cmd);
            }
        }

    }

    public void cancelAll() {

        for (Command command : new ArrayList<>(activeCommands)) {
            cancel(command);
        }

    }





}