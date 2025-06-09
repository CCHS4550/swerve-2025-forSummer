package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.drive.OdometryThread;
import java.util.Queue;

public class PigeonIO implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(Constants.driveConstants.pigeonCanID);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelo = pigeon.getAngularVelocityZWorld();

  public PigeonIO() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Constants.driveConstants.odometryFrequency);
    yawVelo.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = OdometryThread.getInstance().makeTimeQueue();
    yawPositionQueue = OdometryThread.getInstance().registerGenericSignal(yaw::getValueAsDouble);
  }

  @Override
  public void updateInputs(GyroIOInputs in) {
    in.connected = BaseStatusSignal.refreshAll(yaw, yawVelo).equals(StatusCode.OK);
    in.yawRotation2d = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    in.yawRotationSpeed = Units.degreesToRadians((yawVelo.getValueAsDouble()));

    in.yawOdometryTimestamps = yawTimestampQueue.stream().mapToDouble((Double x) -> x).toArray();
    in.yawOdometryPos =
        yawPositionQueue.stream()
            .map((Double x) -> Rotation2d.fromDegrees(x))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
