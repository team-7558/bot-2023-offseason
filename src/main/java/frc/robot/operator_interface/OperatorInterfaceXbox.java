// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterfaceXbox {
  final XboxController m_driverController = new XboxController(0);
  final XboxController m_operatorController = new XboxController(1);

  public default double getTranslateX() {
    return -m_driverController.getLeftY();
  }

  public default double getTranslateY() {
    return -m_driverController.getLeftX();
  }

  public default double getRotate() {
    return -m_driverController.getRightX();
  }

  public default double getOperatorLY() {
    return m_operatorController.getLeftY();
  }

  public default double getOperatorRY() {
    return m_operatorController.getRightY();
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> m_driverController.getBackButton());
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> m_driverController.getStartButton());
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> m_driverController.getLeftBumper());
  }

  public default Trigger getOperatorA() {
    return new Trigger(() -> m_operatorController.getAButton());
  }

  public default Trigger getZeroingButton() {
    return new Trigger(() -> m_operatorController.getBackButton());
  }

  public default Trigger getOperatorStartButton() {
    return new Trigger(() -> m_operatorController.getStartButton());
  }

  public default Trigger getOperatorX() {
    return new Trigger(() -> m_operatorController.getXButton());
  }

  public default Trigger getOperatorY() {
    return new Trigger(() -> m_operatorController.getYButton());
  }

  public default Trigger getOperatorB() {
    return new Trigger(() -> m_operatorController.getBButton());
  }

  public default Trigger getOperatorLeftBumper() {
    return new Trigger(() -> m_operatorController.getLeftBumper());
  }

  public default Trigger getOperatorRightBumper() {
    return new Trigger(() -> m_operatorController.getRightBumper());
  }

  public default Trigger getOperatorRightTrigger() {
    return new Trigger(() -> Math.abs(m_operatorController.getRightTriggerAxis()) > 0.5);
  }

  public default Trigger getOperatorLeftTrigger() {
    return new Trigger(() -> Math.abs(m_operatorController.getLeftTriggerAxis()) > 0.5);
  }

  public default Trigger getDriverRightTrigger() {
    return new Trigger(() -> Math.abs(m_driverController.getRightTriggerAxis()) > 0.5);
  }

  public default Trigger getDriverLeftTrigger() {
    return new Trigger(() -> Math.abs(m_driverController.getLeftTriggerAxis()) > 0.5);
  }
}
