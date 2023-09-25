package frc.robot.commands;

 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team7558.limelightVision.Limelight;
import frc.lib.team7558.limelightVision.LimelightConstants;
import frc.lib.team7558.limelightVision.LimelightConstants.LEDMode;
import frc.lib.team7558.utils.Util;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class LimelightAlign extends CommandBase {



  private Limelight m_limelight;
  private Drivetrain m_drivetrain;
  private LinearFilter m_filter = LinearFilter.singlePoleIIR(0.1, 0.02); 
  private PIDController m_pid;


  
  

  /** Creates a new LimelightAlign. */
  public LimelightAlign(Limelight limelight, Drivetrain drivetrain) {

    addRequirements(drivetrain);
    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_pid = new PIDController(0.045,0.0,0.0001);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.LEDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!Util.inRange(m_limelight.getX(),-2,2)) {
      double tx = m_limelight.getX();
      double txoutput = m_filter.calculate(tx);
      double output = m_pid.calculate(txoutput,0);
      SmartDashboard.putNumber("output", output);
      SmartDashboard.putNumber("txoutput",txoutput);
      m_drivetrain.drive(output * 0.05,0,0,false);
    } else {
      m_drivetrain.drive(0,0,0,false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true);
    m_limelight.setLED(LEDMode.OFF);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Util.inRange(m_limelight.getX(), 0,1);
  }
}
