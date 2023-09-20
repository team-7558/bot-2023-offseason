// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team7558.limelightVision.Limelight;
import frc.lib.team7558.utils.Util;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class LimelightAlign extends CommandBase {

  private Limelight m_limelight;
  private Drivetrain m_drivetrain;
  private PIDController m_pid;
  

  /** Creates a new LimelightAlign. */
  public LimelightAlign(Limelight limelight, Drivetrain drivetrain) {

    addRequirements(drivetrain);
    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_pid = new PIDController (0.06, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.LEDFlash();
  
    m_limelight.LEDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(0,0,m_pid.calculate(m_limelight.getX(),0.5),false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Util.inRange(0.1, 0.2);
    return false;
  }
}
