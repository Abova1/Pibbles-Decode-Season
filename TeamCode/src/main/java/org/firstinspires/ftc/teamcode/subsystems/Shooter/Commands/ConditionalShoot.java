package org.firstinspires.ftc.teamcode.subsystems.Shooter.Commands;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.util.Command.Command;

public class ConditionalShoot implements Command {

    private final double maxDistanceThresh = 100; //GET MAX
    private final double MAX_Velocity = 2580;
    private final double MIN_Veolcity = 2000;
    private final double Zero = 0;
    private double distance;
    private Turret Turret;

    private String name = "Conditional Shooter Command";


    public ConditionalShoot(Turret turret){
        this.Turret = turret;
    }

    @NonNull
    @Override
    public String toString(){
        return "[name=" + name + "]";
    }


    @Override
    public void init() {
        distance = Turret.getPDistance();
    }

    @Override
    public void execute() {

        if(distance > maxDistanceThresh){
            Shooter.setShooterTarget(MAX_Velocity);
        } else {
            Shooter.setShooterTarget(MIN_Veolcity);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean cancelled) {

        if (cancelled) {
            Shooter.setShooterTarget(Zero);
        }

    }
}
