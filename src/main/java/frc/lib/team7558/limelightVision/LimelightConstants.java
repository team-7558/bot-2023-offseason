// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team7558.limelightVision;

/** Constants for Limelight */
public class LimelightConstants {

  /** Field constants */
  public static class FieldConstants { // NOTE: you will have to change these each season
    public static final double fLength = 16.54;
    public static final double fWidth = 8.02;
  }

  /** Limelight Mode (Pipeline) */
  public enum Mode { // NOTE: this is where you associate Pipeline names with their Id's
    DEFAULT(0),
    CONE_LOW(0),
    CONE_HIGH(1);

    int id;

    private Mode(int pipeline) {
      this.id = pipeline;
    }
  }

  /**
   * Limelight LED mode
   *
   * @param DEFAULT - Does whatever is set in the currently selected pipeline
   * @param OFF - forces LEDs off
   * @param BLINK - forces LEDs to blink repeatedly
   * @param ON - forces LEDs on
   */
  public enum LEDMode {
    DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    int id;

    private LEDMode(int mode) {
      this.id = mode;
    }
  }

  /**
   * The camera mode
   *
   * @apiNote - This is diffrent from {@link Mode} and should not be confused with pipelines
   * @param VISION - Uses a pipeline to process vision
   * @param DRIVERCAM - Driver Camera (Increases exposure, disables vision processing)
   */
  public enum camMode {
    VISION(0),
    DRIVERCAM(1);

    int id;

    private camMode(int mode) {
      this.id = mode;
    }
  }
}
