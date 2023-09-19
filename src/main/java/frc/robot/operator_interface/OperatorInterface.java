// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team7558.utils.Util;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {
  final PS4Controller m_driverController = new PS4Controller(0);
  final PS4Controller m_operatorController = new PS4Controller(1);

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
    return new Trigger(() -> m_driverController.getShareButton());
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> m_driverController.getOptionsButton()); // start
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> m_driverController.getL1Button());
  }

  public default Trigger getOperatorA() {
    return new Trigger(() -> m_operatorController.getCrossButton());
  }

  public default Trigger getOperatorPS() {
    return new Trigger(() -> m_operatorController.getPSButton());
  }

  public default Trigger getZeroingButton() {
    return new Trigger(() -> m_operatorController.getShareButton());
  }

  public default Trigger getOperatorStartButton() {
    return new Trigger(() -> m_operatorController.getOptionsButton());
  }

  public default Trigger getOperatorX() {
    return new Trigger(() -> m_operatorController.getSquareButton());
  }

  public default Trigger getOperatorY() {
    return new Trigger(() -> m_operatorController.getTriangleButton());
  }

  public default Trigger getOperatorB() {
    return new Trigger(() -> m_operatorController.getCircleButton());
  }

  public default Trigger getOperatorLeftBumper() {
    return new Trigger(() -> m_operatorController.getL1Button());
  }

  public default Trigger getOperatorRightBumper() {
    return new Trigger(() -> m_operatorController.getR1Button());
  }

  public default Trigger getOperatorRightTrigger() {
    return new Trigger(() -> m_operatorController.getR2Axis() > 0);
  }

  public default Trigger getDriverX() {
    return new Trigger(() -> m_driverController.getCrossButton());
  }

  public default Trigger getDriverCircle() {
    return new Trigger(() -> m_driverController.getCircleButton());
  }

  public default Trigger getDriverTriangle() {
    return new Trigger(() -> m_driverController.getTriangleButton());
  }

  public default Trigger getOperatorLeftTrigger() {
    return new Trigger(() -> m_operatorController.getL2Axis() > 0);
  }

  public default Trigger getDriverRightTrigger() {
    return new Trigger(() -> m_driverController.getR2Axis() > 0);
  }

  public default Trigger getDriverLeftTrigger() {
    return new Trigger(() -> m_driverController.getL2Axis() > 0);
  }

  public default Trigger getDriverRightButton() {
    return new Trigger(() -> m_driverController.getR1Button());
  }

  public default Trigger getOperatorDPad(int pov) {
    return new Trigger(() -> m_operatorController.getPOV() == pov);
  }

  public default Trigger getOperatorDPadSide(int povSide) {
    return new Trigger(() -> Util.inRange(m_operatorController.getPOV() - povSide, 89));
  }

  public default boolean operatorExists() {
    return DriverStation.getJoystickName(1) != null;
  }
}
