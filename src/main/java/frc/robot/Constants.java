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

  public static class ConversionConstants { //lowk copied this from 2025 bot code - might be different

    public static final double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(4 * Math.PI);

    //  rotations of the turn motor to one rotation of the wheel
    // how much of a rotation the wheel turns for one rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1 / 18.75;

    // How many radians the wheel pivots for one full rotation of the turn motor
    public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS =
        Units.rotationsToRadians(TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS);

    public static final double TURN_MOTOR_RADIANS_PER_SECOND =
        TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS / 60.0;

    // 5.36 rotations of the drive motor to one spin of the wheel (L3+)
    // 5.9 rotations of the drive motor to one spin of the wheel (L2+)
    public static final double DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1.0 / 5.36;
    // horizontal distance travelled by one motor rotation
    public static final double HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION =
        (WHEEL_CIRCUMFRENCE * DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS) - 0.0000819403;
    //  - (0.0399 / 5.36);

    public static final double ONE_METER_PER_MOTOR_ROTATIONS = 1 / 16.684;

    public static final double DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR =
        HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION / 60.0;
  }

  public static class MotorConstants { //I (Ruhan) know that these CAN IDs are right - I changed them to be this, dont know about the reverse stuff
    public static final int FRONT_RIGHT_DRIVE = 1;
    public static boolean FRONT_RIGHT_DRIVE_REVERSE = false;

    public static final int FRONT_RIGHT_TURN = 2;
    public static boolean FRONT_RIGHT_TURN_REVERSE = false;

    public static final int BACK_RIGHT_DRIVE = 3;
    public static boolean BACK_RIGHT_DRIVE_REVERSE = false;

    public static final int BACK_RIGHT_TURN = 4;
    public static boolean BACK_RIGHT_TURN_REVERSE = false;

    public static final int BACK_LEFT_DRIVE = 5;
    public static boolean BACK_LEFT_DRIVE_REVERSE = false;

    public static final int FRONT_LEFT_DRIVE = 7;
    public static boolean FRONT_LEFT_DRIVE_REVERSE = false;

    public static final int FRONT_LEFT_TURN = 8;
    public static boolean FRONT_LEFT_TURN_REVERSE = false;

  }
}
