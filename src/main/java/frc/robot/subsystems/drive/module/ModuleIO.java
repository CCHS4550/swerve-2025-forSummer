package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    // variables for drive motor information
    public boolean driveConnected = false;
    public double drivePosRad = 0.0; // drive motor position in radians
    public double driveSpeedVelo = 0.0; // radians per second
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    // variables for turn motor information
    public boolean turnConnected = false;
    public Rotation2d turnPos = new Rotation2d();
    public double turnVelo = 0.0; // radians per second
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePos = new double[] {}; // in radians
    public Rotation2d[] odometryTurnPos = new Rotation2d[] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveOpenLoop(double output) {}

  public default void setTurnOpenLoop(double output) {}

  public default void setDrivevelo(double velo) {} // radians per second

  public default void setTurnPos(Rotation2d rotation) {}
}
