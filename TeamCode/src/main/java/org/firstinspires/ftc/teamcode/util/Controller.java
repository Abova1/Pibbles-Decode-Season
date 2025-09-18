package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Command.Command;
import org.firstinspires.ftc.teamcode.util.Command.CommandScheduler;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/*
This Class is meant to simplify controller inputs
 */

public class Controller {

    private final Gamepad gamepad;
    private final Gamepad previousGamepad = new Gamepad();
    private final CommandScheduler scheduler;

    private final double MIN_thresh = 0.1, MAX_thresh = 1;

    public Controller(Gamepad gamepad, CommandScheduler scheduler) {

        this.gamepad = gamepad;
        this.scheduler = scheduler;

    }
    public void Update(){
        previousGamepad.copy(gamepad);
    }

    public void buttonPressed(BooleanSupplier wasPressed, Supplier<Command> commandSupplier){

        if (wasPressed.getAsBoolean()) {

            Command command = commandSupplier.get();

            if (scheduler.isScheduled(command)) {
                scheduler.cancel(command);
            } else {
                scheduler.schedule(command);
            }

        }

    }

    public void buttonPressed(BooleanSupplier wasPressed, Supplier<Command> commandSupplier, Runnable... actions){

        if (wasPressed.getAsBoolean()) {

            Command command = commandSupplier.get();

            if (scheduler.isScheduled(command)) {
                scheduler.cancel(command);
            } else {
                scheduler.schedule(command);
            }

            for (Runnable action : actions) {
                action.run();
            }

        }

    }

    public void buttonPressed(BooleanSupplier wasPressed, Runnable... actions){
        if(wasPressed.getAsBoolean()){
            for(Runnable action : actions){
                action.run();
            }
        }
    }

    public void triggerPressed(DoubleSupplier wasPressed, double threshold, Supplier<Command> commandSupplier) {

        double thresh = Globals.clamp(threshold, MAX_thresh, MIN_thresh);

        if (wasPressed.getAsDouble() > thresh) {

            Command command = commandSupplier.get();

            if (scheduler.isScheduled(command)) {
                scheduler.cancel(command);
            } else {
                scheduler.schedule(command);
            }

        }
    }

    public void triggerPressed(DoubleSupplier wasPressed, double threshold, Supplier<Command> commandSupplier, Runnable... actions) {

        double thresh = Globals.clamp(threshold, MAX_thresh, MIN_thresh);

        if (wasPressed.getAsDouble() > thresh) {
            Command command = commandSupplier.get();


            if (scheduler.isScheduled(command)) {
                scheduler.cancel(command);
            } else {
                scheduler.schedule(command);
            }

            for(Runnable action : actions){
                action.run();
            }

        }
    }

    public void triggerPressed(DoubleSupplier wasPressed, double threshold, Runnable... actions) {

        double thresh = Globals.clamp(threshold, MAX_thresh, MIN_thresh);

        if (wasPressed.getAsDouble() > thresh) {

            for(Runnable action : actions){
                action.run();
            }

        }
    }

    public void triggerPressed(DoubleSupplier wasPressed, double threshold, Runnable action){

        if(wasPressed.getAsDouble() > threshold){
            action.run();
        }

    }

    /*===============================| Controls |===========================*/

    public double getLy() {
        return gamepad.left_stick_y;
    }

    public double getLx(){
        return gamepad.left_stick_x;
    }

    public double getRx() {
        return gamepad.right_stick_x;
    }
    public double getRy() {
        return gamepad.right_stick_y;
    }


    public boolean a(){
        return gamepad.aWasPressed();
    }

    public boolean b(){
        return gamepad.bWasPressed();
    }
    public boolean x(){
        return gamepad.xWasPressed();
    }
    public boolean y() {
        return gamepad.yWasPressed();
    }

    public boolean RB(){
        return gamepad.rightBumperWasPressed();
    }
    public boolean LB(){
        return gamepad.leftBumperWasPressed();
    }

    public boolean DPR(){
        return gamepad.dpadRightWasPressed();
    }
    public boolean DPL(){
        return gamepad.dpadLeftWasPressed();
    }
    public boolean DPUP(){
        return gamepad.dpadUpWasPressed();
    }
    public boolean DPDOWN(){
        return gamepad.dpadDownWasPressed();
    }

    public double RT(){
        return gamepad.right_trigger;
    }

    public double LT(){
        return gamepad.left_trigger;
    }

    //RISING EDGE CONTROLS

    public boolean REa(){
        return gamepad.a && !previousGamepad.a;
    }

    public boolean REb(){
        return gamepad.b && !previousGamepad.b;
    }
    public boolean REx(){
        return gamepad.x && !previousGamepad.x;
    }
    public boolean REy() {
        return gamepad.y && !previousGamepad.y;
    }

    public boolean RERB(){
        return gamepad.right_bumper && !previousGamepad.right_bumper;
    }
    public boolean RELB(){
        return gamepad.left_bumper && !previousGamepad.left_bumper;
    }

    public boolean REDPR(){
        return gamepad.dpad_right && !previousGamepad.dpad_right;
    }
    public boolean REDPL(){
        return gamepad.dpad_left && !previousGamepad.dpad_left;
    }
    public boolean REDPUP(){
        return gamepad.dpad_up && !previousGamepad.dpad_up;
    }
    public boolean REDPDOWN(){
        return gamepad.dpad_down && !previousGamepad.dpad_down;
    }



}