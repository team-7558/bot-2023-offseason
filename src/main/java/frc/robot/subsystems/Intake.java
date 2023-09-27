// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class Intake extends SubsystemBase {

  private Solenoid m_piston = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.kIntakePiston);
  private CANSparkMax m_motor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor,MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {

  }

  public void intake() {
    m_piston.set(true);
    m_motor.set(0.4);
  }

  public void stopIntake() {
    m_motor.set(0);
    m_piston.set(false);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
