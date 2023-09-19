// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean TUNING_MODE = false;

  /** Intake Constants */
  public static class IntakeConstants {
    public static final int kIntakeMotor = 10; // TODO: Set intake motor port
    public static final int kIntakePiston = 15; // TODO: Set intake piston port
    public static final double kIntakeMotorSpeed =
        0.5; // 0.45346; // 0.35; // 0.5; // TODO: set intake motor speed to something faster later
  }

  public static class ElevatorConstants { // TODO: Set all of these
    public static final double kGearing = 16; // to 1, before 28:1

    public static final int kTimeoutMs = 30;
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kElevatorMotor = 16;
    public static final double kP = 0.06;
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double kF = 0.043;
    public static final double kMinError = 250;

    // fast
    // public static final double kCruise = 3000*kGearing;
    // public static final double kAcc = 2000*kGearing;

    public static final double kCruise = 3000 * kGearing;
    public static final double kAcc = 4000 * kGearing;
    public static final int kSmoothing = 0;

    public static final int kHallEffectPort = 4;

    public static final double kZeroingSpeed = -0.05;

    public static class Presets { // 200 Units per Inch :)
      public static final double BOTTOM = 0.0 * kGearing;
      public static final double MIDDLE = (3407.32 + 500 + 200) * kGearing;
      public static final double HIGH =
          (7285.71 + 450) * kGearing; // (7285.71 + 250) * kGearing; // 7285.71*kGearing;
      public static final double HUMAN = MIDDLE - 200 + (2300 * kGearing);
      public static final double PREINTAKE = (685.71 + 250) * kGearing;
      public static final double POSTINTAKE = (678.57 + 250) * kGearing;
      // at 28
      // 0
      // 95405
      // 204000
      // 19200
      // 19000
    }
  }

  /** Grabber Constants */
  public static class GrabberConstants {
    public static final int kSwipeyMotor = 11;
    public static final int kGrabbyMotor = 12;
    public static final int kTwistyMotor = 13;
    public static final int kGrabberExtendSolenoid = 13; // used to be 14 but for now does not exist
    public static final int kGrabberGripSolenoid = 14;

    public static final double kGrabbySpeed = 0.6;
    public static final double kGrabbySpitSpeed = 0.567; // 0.7; // 0.375;

    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMotorMinSpeed = -0.5;
    public static final double kMotorMaxSpeed = 0.7; // 0.7;
    public static final double kDropSpitSpeed = 0.14;

    public static final double kZeroingSpeed = 0.05;
    public static final int kHallEffectPort = 5;
    public static final double kZero = 41.5; // TOOO: ?
    public static final double kZeroPos = 0;
    public static final double kZeroOffset = 4 + 3; // 1.8;

    public static final double kRotationsPerDegree = 4;
    public static final double kWristRotationsPerDegree = 5.8;

    public static class Presets {
      public static final double PICKUP = 81.2 + kZeroOffset;
      public static final double LOW = 11.78 + kZeroOffset;
      public static final double HIGH = 150 + kZeroOffset; // 150;
      public static final double HUMAN = 118 - 5.5 + kZeroOffset;
      public static final double CUBE_INTAKE = 69.69 + kZeroOffset - 4; // 67.654321;//69.69;
      public static final double CONE_INTAKE = 61 + kZeroOffset; // 67.654321;//69.69;
      public static final double CLEARANCE = 123.456 + kZeroOffset; // 131.313;//123.456; //135]
      public static final double SPIT = 76.54321;
    }
  }

  public static class VisionConstants {
    public static final double kXOffsetHigh = 8.0; // todo set this
    public static final double kXOffsetLow = 14.4;
  }

  private static final RobotType ROBOT = RobotType.ROBOT_DEFAULT;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  // FIXME: update for various robots
  public enum RobotType {
    ROBOT_2022_SIERRA,
    ROBOT_2023_MK4I,
    ROBOT_DEFAULT,
    ROBOT_SIMBOT
  }

  // FIXME: update for various robots
  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_DEFAULT;
      } else {
        return ROBOT;
      }
    } else {
      return ROBOT;
    }
  }

  // FIXME: update for various robots
  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_DEFAULT:
      case ROBOT_2022_SIERRA:
      case ROBOT_2023_MK4I:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public static final double LOOP_PERIOD_SECS = 0.02;
}
