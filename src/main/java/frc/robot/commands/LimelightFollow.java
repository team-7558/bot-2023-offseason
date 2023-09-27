// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team7558.limelightVision.Limelight;
import frc.lib.team7558.limelightVision.LimelightConstants.LEDMode;
import frc.lib.team7558.utils.Util;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class LimelightFollow extends CommandBase {



  private Limelight m_limelight;
  private final double deg_rad = Math.PI / 180;
  private Drivetrain m_drivetrain;
  private Timer m_timer = new Timer();
  private TrapezoidProfile m_motionProfile;
  private LinearFilter m_filter = LinearFilter.singlePoleIIR(0.1, 0.02); 
  private PIDController m_pid;


  
  

  /** Creates a new LimelightAlign. */
  public LimelightFollow(Limelight limelight, Drivetrain drivetrain) {

    addRequirements(drivetrain);
    this.m_drivetrain = drivetrain;
    this.m_limelight = limelight;
    this.m_pid = new PIDController(0.045,0.0,0.0001);



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.LEDOn();
    this.m_motionProfile = new TrapezoidProfile(
      new Constraints(20, 5),
      new TrapezoidProfile.State(0,0),
      new TrapezoidProfile.State(m_filter.calculate(m_limelight.getX()), 0.0));
    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mGraphVelocity = m_motionProfile.calculate(m_timer.get()).velocity * deg_rad;
    SmartDashboard.putNumber("tx", m_limelight.getX());
    SmartDashboard.putNumber("mGraphVelocity", mGraphVelocity);

    m_drivetrain.drive(0,0,mGraphVelocity,false);
    // if(!Util.inRange(m_limelight.getX(),-2,2)) {
    //   double tx = m_limelight.getX();
    //   double txoutput = m_filter.calculate(tx);
    //   double output = m_pid.calculate(txoutput,0);
    //   SmartDashboard.putNumber("output", output);
    //   SmartDashboard.putNumber("txoutput",txoutput);
    //   m_drivetrain.drive(0,0,output,false);
    // } else {
    //   m_drivetrain.drive(0,0,0,false);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_drivetrain.drive(0, 0, 0, true);
    m_limelight.setLED(LEDMode.OFF);
    this.m_motionProfile = new TrapezoidProfile(
      new Constraints(20, 5),
      new TrapezoidProfile.State(0,0),
      new TrapezoidProfile.State(m_filter.calculate(m_limelight.getX()), 0.0));
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Util.inRange(m_limelight.getX(), 0,1);
  }
}
