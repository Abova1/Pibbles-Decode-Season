package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Custom.Turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Intake.GobuildaIntake;
import org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gobuilda.Intake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooters;
import org.firstinspires.ftc.teamcode.util.Command.*;


public class TeleHandler {

    private RobotState state = RobotState.REGULAR;
    private Shooters shooter;
    private Turret turret;
    private GobuildaIntake intake;
    private Controller Driver, Operator;
    private Object[] subsystems;

    private CommandScheduler scheduler;
    private Command maxVeloCommand;
    private Command minVeloCommand;
    private Command IntakeCommand;
    private Command OuttakeCommand;


    public TeleHandler(Controller driver, CommandScheduler scheduler, Object... subsystems){

        this.Driver = driver;
        this.scheduler = scheduler;
        this.subsystems = subsystems;

        for (Object subsystem : subsystems) {

            if (subsystem instanceof Shooters) {
                this.shooter = (Shooters) subsystem;
            }

            if (subsystem instanceof Turret) {
                this.turret = (Turret) subsystem;
            }

            if(subsystem instanceof GobuildaIntake){
                this.intake = (GobuildaIntake) subsystem;
            }

        }

        maxVeloCommand = new maxVelocityCommand();
        minVeloCommand = new minVelocityCommand();
        IntakeCommand = new IntakeCommand();
        OuttakeCommand = new OuttakeCommand();

    }

    public RobotState getState(){
        return state;
    }

    public void TeleOp (){

        if(shooter != null){
            shooter.run();
        }

        if(turret != null){
            turret.run();
        }

        if(intake != null){
            intake.run();
        }

        switch (state){

            case REGULAR:

                Driver.buttonPressed(Driver::a, () -> IntakeCommand);
                Driver.buttonPressed(Driver::b, () -> OuttakeCommand);

            break;
        }

    }

}