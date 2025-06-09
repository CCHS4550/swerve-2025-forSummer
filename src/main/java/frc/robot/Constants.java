// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class driveConstants {
    public static final int pigeonCanID = 9999;
    public static final double odometryFrequency = 120.0;

    public static final Rotation2d frontLeftOffset = Rotation2d.fromDegrees(99999);
    public static final Rotation2d frontRightOffset = Rotation2d.fromDegrees(99999);
    public static final Rotation2d backLeftOffset = Rotation2d.fromDegrees(99999);
    public static final Rotation2d backRightOffset = Rotation2d.fromDegrees(99999);

    public static final int frontLeftDriveCanID = 9999999;
    public static final int frontRightDriveCanId = 999999;
    public static final int backLeftDriveCanId = 99999999;
    public static final int backRightDriveCanID = 9999999;

    public static final int frontLeftTurnCanID = 9999999;
    public static final int frontRightTurnCanId = 999999;
    public static final int backLeftTurnCanId = 99999999;
    public static final int backRightTurnCanID = 9999999;

    public static final boolean frontLeftTurnInverted = false;
    public static final boolean frontRightTurnInverted = false;
    public static final boolean backLeftTurnInverted = false;
    public static final boolean backRightTurnInverted = false;

    public static final boolean frontLeftTurnEncoderInverted = true;
    public static final boolean frontRightTurnEncoderInverted = true;
    public static final boolean backLeftTurnEncoderInverted = true;
    public static final boolean backRightTurnEncoderInverted = true;

    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 99999999;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    public static final double turnEncoderPositionFactor =
        2 * Math.PI; // converting rotations to radians
    public static final double turnEncoderVeloFactor = (2 * Math.PI) / 60.0; // RPM to radians/sec

    // PID turn constants
    public static final double turnKp = 2.0; // stand in values from mechanical advantage
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0; // stand in values from mechanical advantage
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // radians

    public static final boolean frontLeftDriveInverted = false;
    public static final boolean frontRightDriveInverted = false;
    public static final boolean backLeftDriveInverted = false;
    public static final boolean backRightDriveInverted = false;

    public static final int driveMotorCurrentLimit = 50;
    public static final double driveMotorReduction = 999999999999.9;

    public static final boolean frontLeftDriveEncoderInverted = true;
    public static final boolean frontRightDriveEncoderInverted = true;
    public static final boolean backLeftDriveEncoderInverted = true;
    public static final boolean backRightDriveEncoderInverted = true;

    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // rotations to radians
    public static final double driveEncoderVeloFactor =
        (2 * Math.PI) / 60 / driveMotorReduction; // rpm to rad/sec

    // Drive PID configuration values are placeholders from mechanical advantage
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    public static double wheelRadius = 99999; // in meters
  }
}
