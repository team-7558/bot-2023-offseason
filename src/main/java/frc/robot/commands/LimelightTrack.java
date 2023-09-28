// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team7558.limelightVision.Limelight;
import frc.lib.team7558.limelightVision.LimelightConstants;
import frc.lib.team7558.utils.Util;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;



public class LimelightTrack extends CommandBase {


  public static enum TrackingState {
    FORWARD,
    SIDEWAYS,
    FINISHED,
    CANCELLED
  }

  private final double deg_rad = Math.PI / 180;

  private Drivetrain m_drivetrain;
  private Limelight m_limelight;

  // ALL UNITS IN METERS
  private final double kLimelightHeight = Units.inchesToMeters(21.39);
  private final double kPoleHighHeight = Units.inchesToMeters(43.5);
  private double kLimelightAngle = 10;
  private double kDistanceOffset = -0.3;
  private double kTargetDistance = 1.18;
  private double kxVelocity = 0.2;
  private double kyVelocity = 0.2;

  private TrackingState m_state;

  private LinearFilter m_tyFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter m_txFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private PIDController m_alignController;
  private PIDController m_movementController;


  /** Creates a new LimelightTrack. */
  public LimelightTrack(Drivetrain drive, Limelight lime) {

    addRequirements(drive);
    this.m_limelight = lime;
    this.m_drivetrain = drive;
    this.m_alignController = new PIDController(0.045, 0, 0.001);
    SmartDashboard.putNumber("xVelocity", kxVelocity);
    SmartDashboard.putNumber("yVelocity", kyVelocity);
    SmartDashboard.putNumber("targetDistance", kTargetDistance);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_state = TrackingState.SIDEWAYS;
    m_drivetrain.zeroGyroscope();
    m_limelight.LEDOn();

    System.out.println("init");
    this.m_movementController = new PIDController(0.045, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_state == TrackingState.FORWARD) {
      double distance = getDistanceFromTarget();
      SmartDashboard.putNumber("calculatedDistance", distance);
      double targetDistance = SmartDashboard.getNumber("targetDistance", 1.5);
      if(distance > targetDistance) {
        double xVelocity = SmartDashboard.getNumber("xVelocity",0.05);
        m_drivetrain.drive(xVelocity,0,0, false);
      } else if(distance != 0) {
        m_drivetrain.drive(0, 0, 0, false);
      }
    } else if(m_state == TrackingState.SIDEWAYS) {
      double txDistance = m_txFilter.calculate(m_limelight.getX());
      if(!Util.inRange(txDistance, -0.5,0.5)) {
        double yVelocity = SmartDashboard.getNumber("yVelocity",0.05);
        m_drivetrain.drive(0,yVelocity,0, false);
      } else {
        m_drivetrain.drive(0, 0, 0, false);
      }
    }

  }


  public double getDistanceFromTarget() {
    double angleToGoalDegrees = m_tyFilter.calculate(m_limelight.getY()) + kLimelightAngle;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distance = (kPoleHighHeight - kLimelightHeight) / Math.tan(angleToGoalRadians);
    return distance + kDistanceOffset;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false);
    if(interrupted) {
      this.m_state = TrackingState.CANCELLED;
    } else {
      this.m_state = TrackingState.FINISHED;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_state == TrackingState.FORWARD) {
      double targetDistance = SmartDashboard.getNumber("targetDistance", 1.5);
      return getDistanceFromTarget() <= targetDistance;
    } else if(m_state == TrackingState.SIDEWAYS) {
      if(Util.inRange(m_txFilter.calculate(m_limelight.getX()),-0.5,0.5)) {
        this.m_state = TrackingState.FORWARD;
      }
    }
    return false;
  }
}
