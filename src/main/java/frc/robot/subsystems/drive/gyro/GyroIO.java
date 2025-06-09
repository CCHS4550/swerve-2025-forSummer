package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawRotation2d = new Rotation2d();
    public double yawRotationSpeed = 0.0; // rad per sec
    public double[] yawOdometryTimestamps = new double[] {};
    public Rotation2d[] yawOdometryPos = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs in) {}
}
