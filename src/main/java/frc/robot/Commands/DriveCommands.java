package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
    private static final double DEADBAND = 0.0;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELO = 8.0;
    private static final double ANGLE_MAX_ACCEL = 20.0;
    private static final double FF_START_DELAY = 2.0; //sec
    private static final double FF_RAMP_RATE = 0.1; //volts per sec
    private static final double WHEEL_RADIUS_MAX_VELO = 0.25; //radians per sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // radians / sec^2  

    private DriveCommands() {}

    private static Translation2d turnJoystickInputToLinearVelo(double x, double y){
        
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y,x));
        
        linearMagnitude = linearMagnitude * linearMagnitude;

        return new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
    }

    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, DoubleSupplier multiplierSupplier){
        return Commands.run(
            () -> {
                Translation2d linearVelo = turnJoystickInputToLinearVelo(xSupplier.getAsDouble() * multiplierSupplier.getAsDouble(), ySupplier.getAsDouble() * multiplierSupplier.getAsDouble());

                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                ChassisSpeeds speeds = new ChassisSpeeds(
                    linearVelo.getX() * drive.getMaxLinearSpeed(), 
                    linearVelo.getY() * drive.getMaxLinearSpeed(), 
                    omega * drive.getMaxAngularSpeed()
                    );
                
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(speeds, 
                        Constants.isFlipped() 
                            ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation())
                );

            }, 
            drive);
    }
}


