// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team7558.limelightVision.Limelight;
import frc.lib.team7558.utils.Util;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;



public class LimelightTrack extends CommandBase {

  private final double deg_rad = Math.PI / 180;

  private Drivetrain m_drivetrain;
  private Limelight m_limelight;
  
  private Timer m_timer;
  private LinearFilter m_depthFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter m_txFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private PIDController m_alignController;
  private TrapezoidProfile m_movementController;

  /** Creates a new LimelightTrack. */
  public LimelightTrack(Drivetrain drive, Limelight lime) {
    RobotConfig config = RobotContainer.getInstance().getConfig();
    addRequirements(drive);
    this.m_limelight = lime;
    double DEPTH_VAR_CHANGE_LATER = 0;
    this.m_drivetrain = drive;
    this.m_alignController = new PIDController(0.045, 0, 0.001);
    this.m_movementController = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        (config.getRobotMaxVelocity() * deg_rad) * 0.025,
        (config.getAutoMaxAcceleration() * deg_rad) * 0.025),
      new TrapezoidProfile.State(0, 0),
      new TrapezoidProfile.State(m_depthFilter.calculate(DEPTH_VAR_CHANGE_LATER),0));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.LEDOn();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double depthVelocity = m_movementController.calculate(m_timer.get()).velocity;
    double turnVelocity = m_alignController.calculate(m_txFilter.calculate(depthVelocity),m_limelight.getX());

    m_drivetrain.drive(0, depthVelocity, turnVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
