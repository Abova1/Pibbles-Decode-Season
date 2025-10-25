package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Chamber.Chamber;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands.CloseCommand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands.ConditionalShoot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands.FarCommand;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.util.Command.*;

public class TeleHandler {

    private RobotState state = RobotState.REGULAR;
    private Controller Driver, Operator;

    private Shooter shooter;
    private Chamber chamber;
    private Intake intake;
    private Turret turret;


    private Object[] subsystems;
    private CommandScheduler scheduler;

    //Shooter Commands
    private Command conditionalShoot;
    private Command FarShoot;
    private Command CloseShoot;

    //Intake Commands
    private Command IntakeCommand;
    private Command OuttakeCommand;

    //Special Commands
    private Command Shoot;
    private Command diffShoot;


    public TeleHandler(Controller driver, CommandScheduler scheduler, Object... subsystems){

        this.Driver = driver;
        this.scheduler = scheduler;
        this.subsystems = subsystems;

        for (Object subsystem : subsystems) {

            if (subsystem instanceof Shooter) {
                this.shooter = (Shooter) subsystem;
            }
            if (subsystem instanceof Chamber) {
                this.chamber = (Chamber) subsystem;
            }
            if (subsystem instanceof Intake) {
                this.intake = (Intake) subsystem;
            }
            if (subsystem instanceof Turret) {
                this.turret = (Turret) subsystem;
            }


        }

        CloseShoot = new CloseCommand();
        FarShoot = new FarCommand();

        IntakeCommand = new IntakeCommand();
        OuttakeCommand = new OuttakeCommand();

        Shoot = new ParallelCommand(
                conditionalShoot,
                IntakeCommand
        );

        diffShoot = new ParallelCommand(
                CloseShoot,
                IntakeCommand
        );

    }

    public RobotState getState(){
        return state;
    }

    public void TeleOp (){

        if(shooter != null){
            shooter.run();
        }

        if(intake != null){
            intake.run();
        }

        if(chamber != null){
            chamber.chamberIn();
        }

        switch (state){

            case REGULAR:
                Driver.buttonPressed(Driver::a, ()-> Shoot);
                Driver.buttonPressed(Driver::b, ()-> IntakeCommand);
                Driver.buttonPressed(Driver::x, ()-> OuttakeCommand);
            break;
        }

    }

    public void TestTeleOp(){

        if(shooter != null){
            shooter.run();
        }

        if(intake != null){
            intake.run();
        }

        switch (state){

            case REGULAR:

                Driver.buttonPressed(Driver::a, ()-> diffShoot);
                Driver.buttonPressed(Driver::b, ()-> IntakeCommand);
                Driver.buttonPressed(Driver::x, ()-> OuttakeCommand);
                break;
        }

    }

}