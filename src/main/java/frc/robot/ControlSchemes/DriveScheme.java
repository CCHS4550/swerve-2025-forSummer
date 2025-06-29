package frc.robot.controlschemes;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriveScheme {
    private static DoubleSupplier driveSpeedModifier = ()-> 0.4;
    private static IntSupplier counter = ()-> 3;
    public static void configure(Drive drive, CommandXboxController controller){
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive, 
                ()-> -controller.getLeftY()*driveSpeedModifier.getAsDouble(), 
                ()-> -controller.getLeftX()*driveSpeedModifier.getAsDouble(), 
                ()-> - controller.getRightX(),
                ()-> calcSpeedMod().getAsDouble())
                );
        configureButtons(controller);
    }
    private static void configureButtons(CommandXboxController controller){
        controller.rightBumper().onTrue(runOnce(() -> countup()));
    }
    private static void setFastMode() {
        driveSpeedModifier = () -> .7;
    }
    private static void setNormalMode() {
        driveSpeedModifier = () -> 0.4;
    }
    private static void setSlowMode() {
        driveSpeedModifier = () -> 0.05;
    }
    private static void countup(){
        counter = ()-> counter.getAsInt() + 1;
    }
    private static void resetCount(){
        counter = ()-> (counter.getAsInt() % 3) + 3;
    }
    private static DoubleSupplier calcSpeedMod(){
        if(counter.getAsInt() % 3 == 0){
            runOnce(() -> setNormalMode());
        }
        else if(counter.getAsInt() % 3 == 1){
            runOnce(() -> setFastMode());
        }
        else{
            runOnce(() -> setSlowMode());
        }
        if(counter.getAsInt() > 100){
            runOnce(()-> resetCount());
        }
        return driveSpeedModifier;
    }
}
