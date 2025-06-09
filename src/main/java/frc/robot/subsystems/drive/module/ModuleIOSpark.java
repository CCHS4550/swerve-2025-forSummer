package frc.robot.subsystems.drive.module;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.OdometryThread;
import frc.robot.util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d rotationOffset;

  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final Queue<Double> timeQueue;
  private final Queue<Double> drivePosQueue;
  private final Queue<Double> turnPosQueue;

  private final Debouncer driveDebouncer = new Debouncer(0.5);
  private final Debouncer turnDebouncer = new Debouncer(0.5);

  public ModuleIOSpark(int module) {
    rotationOffset =
        switch (module) {
          case 0 -> Constants.driveConstants.frontLeftOffset;
          case 1 -> Constants.driveConstants.frontRightOffset;
          case 2 -> Constants.driveConstants.backLeftOffset;
          case 3 -> Constants.driveConstants.backRightOffset;
          default -> new Rotation2d();
        };
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.driveConstants.frontLeftTurnCanID;
              case 1 -> Constants.driveConstants.frontRightTurnCanId;
              case 2 -> Constants.driveConstants.backLeftTurnCanId;
              case 3 -> Constants.driveConstants.backRightTurnCanID;
              default -> 0;
            },
            MotorType.kBrushless);
    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.driveConstants.frontLeftDriveCanID;
              case 1 -> Constants.driveConstants.frontRightDriveCanId;
              case 2 -> Constants.driveConstants.backLeftDriveCanId;
              case 3 -> Constants.driveConstants.backRightDriveCanID;
              default -> 0;
            },
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder(); // don't need an absolute encoder for drive
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // turn motor config
    var turnConfig = new SparkMaxConfig();
    switch (module) {
      case 0 -> turnConfig.inverted(Constants.driveConstants.frontLeftTurnInverted);
      case 1 -> turnConfig.inverted(Constants.driveConstants.frontRightTurnInverted);
      case 2 -> turnConfig.inverted(Constants.driveConstants.backLeftTurnInverted);
      case 3 -> turnConfig.inverted(Constants.driveConstants.backRightTurnInverted);
      default -> turnConfig.inverted(false);
    }
    switch (module) {
      case 0 -> turnConfig.absoluteEncoder.inverted(
          Constants.driveConstants.frontLeftTurnEncoderInverted);
      case 1 -> turnConfig.absoluteEncoder.inverted(
          Constants.driveConstants.frontRightTurnEncoderInverted);
      case 2 -> turnConfig.absoluteEncoder.inverted(
          Constants.driveConstants.backLeftTurnEncoderInverted);
      case 3 -> turnConfig.absoluteEncoder.inverted(
          Constants.driveConstants.backRightTurnEncoderInverted);
    }
    turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.driveConstants.turnMotorCurrentLimit)
        .voltageCompensation(12);
    turnConfig
        .absoluteEncoder
        .positionConversionFactor(Constants.driveConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(Constants.driveConstants.turnEncoderVeloFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            Constants.driveConstants.turnPIDMinInput, Constants.driveConstants.turnPIDMaxInput)
        .pidf(Constants.driveConstants.turnKp, 0.0, Constants.driveConstants.turnKd, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(
            (int) (1000.0 / Constants.driveConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.makeItWork(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    var driveConfig = new SparkMaxConfig();
    switch (module) {
      case 0 -> driveConfig.inverted(Constants.driveConstants.frontLeftDriveInverted);
      case 1 -> driveConfig.inverted(Constants.driveConstants.frontRightDriveInverted);
      case 2 -> driveConfig.inverted(Constants.driveConstants.backLeftDriveInverted);
      case 3 -> driveConfig.inverted(Constants.driveConstants.backRightDriveInverted);
      default -> driveConfig.inverted(false);
    }
    switch (module) {
      case 0 -> driveConfig.encoder.inverted(
          Constants.driveConstants.frontLeftDriveEncoderInverted);
      case 1 -> driveConfig.encoder.inverted(
          Constants.driveConstants.frontRightDriveEncoderInverted);
      case 2 -> driveConfig.encoder.inverted(Constants.driveConstants.backLeftDriveEncoderInverted);
      case 3 -> driveConfig.encoder.inverted(
          Constants.driveConstants.backRightDriveEncoderInverted);
    }
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.driveConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(Constants.driveConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(Constants.driveConstants.driveEncoderVeloFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Constants.driveConstants.driveKp, 0.0, Constants.driveConstants.driveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.driveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    makeItWork(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    makeItWork(
        driveSpark,
        5,
        () -> driveEncoder.setPosition(0.0)); // reset the sparkmax encoder b/c its not absolute

    // make the queues
    timeQueue = OdometryThread.getInstance().makeTimeQueue();
    drivePosQueue =
        OdometryThread.getInstance().registerSparkSignal(driveSpark, driveEncoder::getPosition);
    turnPosQueue =
        OdometryThread.getInstance().registerSparkSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // drive
    SparkUtil.stickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePosRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveSpeedVelo = value);
    ifOK(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveDebouncer.calculate(!SparkUtil.stickyFault);

    // turn
    SparkUtil.stickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPos = new Rotation2d(value).minus(rotationOffset));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelo = value);
    ifOK(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnDebouncer.calculate(!SparkUtil.stickyFault);

    // odometry
    inputs.odometryTimestamps = timeQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePos = drivePosQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPos =
        turnPosQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(rotationOffset))
            .toArray(Rotation2d[]::new);
    timeQueue.clear();
    drivePosQueue.clear();
    turnPosQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDrivevelo(double velo) { // radians per second
    double ffvolts =
        Constants.driveConstants.driveKs * Math.signum(velo)
            + Constants.driveConstants.driveKv * velo;
    driveController.setReference(
        velo, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffvolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPos(Rotation2d rotation) {
    double setPoint =
        MathUtil.inputModulus(
            rotation.plus(rotationOffset).getRadians(),
            Constants.driveConstants.turnPIDMinInput,
            Constants.driveConstants.turnPIDMaxInput);
    turnController.setReference(setPoint, ControlType.kPosition);
  }
}
