// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team7558.limelightVision.Limelight;

public class LimelightToggle extends CommandBase {

  private Limelight m_limelight;
  /** Creates a new LimelightToggle. */
  public LimelightToggle(Limelight limelight) {

    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_limelight.led_state) {
      m_limelight.LEDOn();
    } else {
      m_limelight.LEDOff();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
