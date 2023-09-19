// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team7558.limelightVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.team7558.limelightVision.LimelightConstants.LEDMode;
import frc.lib.team7558.limelightVision.LimelightConstants.Mode;
import frc.lib.team7558.limelightVision.LimelightConstants.camMode;

/** Add your docs here. */
public class Limelight {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry pl = table.getEntry("pipeline");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry tl = table.getEntry("tl");
  private NetworkTableEntry cl = table.getEntry("cl");
  private NetworkTableEntry ledMode = table.getEntry("ledMode");
  private NetworkTableEntry botpose_ts = table.getEntry("botpose_targetspace");
  private NetworkTableEntry botpose_fs = table.getEntry("botpose");
  private NetworkTableEntry cm = table.getEntry("camMode");
  private NetworkTableEntry tid = table.getEntry("tid");
  private NetworkTableEntry tcornxy = table.getEntry("tcornxy");
  private NetworkTableEntry getpipe = table.getEntry("getpipe");

  public static double fieldLong = LimelightConstants.FieldConstants.fLength;
  public static double fieldShort = LimelightConstants.FieldConstants.fWidth;

  private static Transform2d globalFieldToBlueAlliance =
      new Transform2d(
          new Pose2d(0, 0, new Rotation2d(0)),
          new Pose2d(Limelight.fieldLong / 2, Limelight.fieldShort / 2, new Rotation2d(0)));
  private static Transform2d globalFieldToRedAlliance =
      new Transform2d(
          new Pose2d(0, 0, new Rotation2d(0)),
          new Pose2d(
              -Limelight.fieldLong / 2, -Limelight.fieldShort / 2, new Rotation2d(Math.PI / 2)));

  Mode mode;

  /** Creates a new Limelight using the default pipeline */
  public Limelight() {
    this(Mode.DEFAULT);
  }

  /**
   * Creates a new limelight using a given pipeline
   *
   * @param startingMode - The {@link Mode} to start in
   */
  public Limelight(Mode startingMode) {
    this.mode = startingMode;
    this.pipeline(this.mode);
  }

  /**
   * @return The currently set limelight pipeline ID
   */
  public double getPipeline() {
    return getpipe.getDouble(0);
  }

  /**
   * @return X value of the contours corner
   */
  public double getCornerX() {
    double[] corners = tcornxy.getDoubleArray(new double[8]);
    if (corners.length > 0) {
      return corners[0];
    }
    return -1;
  }

  /**
   * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
   */
  public double getX() {
    return tx.getDouble(0.0);
  }

  /**
   * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
   */
  public double getY() {
    return ty.getDouble(0.0);
  }

  /**
   * @return Target Area (0% of image to 100% of image)
   */
  public double getArea() {
    return ta.getDouble(0.0);
  }

  /**
   * @return The full latency of the limelight including pipeline processing time
   */
  public double getLatency() {
    return tl.getDouble(999) + cl.getDouble(999);
  }

  /**
   * @return ID of the primary in-view AprilTag
   */
  public double getAprilTagId() {
    return tid.getDouble(-1.0);
  }

  /**
   * @return 3D transform of the robot in the coordinate system of the primary in-view AprilTag
   *     (array (6))
   */
  public double[] getBotPoseTs() {
    return botpose_ts.getDoubleArray(new double[0]);
  }

  /**
   * @return 3D transform of the robot in the coordinate system of the field (array (6))
   */
  public double[] getBotPoseFs() {
    return botpose_fs.getDoubleArray(new double[7]);
  }

  /**
   * @return transform of the robot in the coordinate system of the field in the form of a WPILib
   *     {@link Pose2d}
   */
  public Pose2d getBotPose2dFs() {
    double[] pose3d = getBotPoseFs();
    return new Pose2d(pose3d[0], pose3d[1], Rotation2d.fromDegrees(pose3d[5]));
  }

  /**
   * Gets the bots {@link Pose2d}, but translated for the correct alliance relative to the field
   *
   * @param alliance - The alliance to translate the coordinate system for (1 = blue, 0 = red)
   * @return transform of the robot in the coordinate system of the field in the form of a WPILib
   *     {@link Pose2d}
   */
  public Pose2d getBotPose2dAlliance(double alliance) {
    Pose2d pose = getBotPose2dFs();
    return alliance == 1
        ? pose.transformBy(globalFieldToBlueAlliance)
        : pose.transformBy(globalFieldToRedAlliance);
  }

  /**
   * @return Whether the limelight has any valid targets
   */
  public boolean hasTarget() {
    return tv.getDouble(0.0) == 1 ? true : false;
  }

  /**
   * Set the LED mode for the limelight
   *
   * @param led - {@link LEDMode} the LED mode to set the limelight to
   */
  public Limelight setLED(LEDMode led) {
    ledMode.setDouble(led.id);
    return this;
  }

  /** Turn on LEDs */
  public void LEDOn() {
    setLED(LEDMode.ON);
  }

  /** Turn off LEDs */
  public void LEDOff() {
    setLED(LEDMode.OFF);
  }

  /** Turn blink the LEDs */
  public void LEDFlash() {
    setLED(LEDMode.BLINK);
  }

  /**
   * Set the pipeline for the limelight to use
   *
   * @param pipeline - {@link Mode} to use
   */
  public Limelight pipeline(Mode pipeline) {
    return setMode(pipeline);
  }

  /**
   * Set the pipeline for the limelight to use
   *
   * @param pipeline - {@link Mode} to use
   */
  public Limelight setMode(Mode pipeline) {
    mode = pipeline;
    pl.setDouble(pipeline.id);
    return this;
  }

  /** Enables Driver Camera (Increases exposure, disables vision processing) */
  public Limelight enableCam() {
    cm.setDouble(camMode.DRIVERCAM.id);
    return this;
  }

  /** Disables Driver Camera */
  public Limelight disableCam() {
    cm.setDouble(camMode.VISION.id);
    return this;
  }

  /**
   * @return If the limelight is currently set to be a driver camera
   */
  public boolean isDriverCamEnabled() {
    return cm.getDouble(0) == 1;
  }
}
