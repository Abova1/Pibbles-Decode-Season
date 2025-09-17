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

    public void run() throws InterruptedException {

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

    public void cancel(Command command) {
        if(activeCommands.contains(command)){
            command.end(true);
            activeCommands.remove(command);
        }
    }

    public void cancelAll() {

        for (Command cmd : new ArrayList<>(activeCommands)) {
            cancel(cmd);
            if (cmd.isFinished()) {
                activeCommands.remove(cmd);
            }
        }

    }





}