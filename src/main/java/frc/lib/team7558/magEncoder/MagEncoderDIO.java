package frc.lib.team7558.magEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** MagEncoder class for ctre Mag Encoders used through the RoboRio's DIO ports */
public class MagEncoderDIO {
  private final DutyCycleEncoder encoder;

  /**
   * MagEncoder class for ctre Mag Encoders used through the RoboRio's DIO ports
   *
   * @param id - The ID of the DIO port the MagEncoder is plugged into
   */
  public MagEncoderDIO(int id) {
    encoder = new DutyCycleEncoder(id);

    encoder.setDutyCycleRange(0, 4096);
    encoder.setDistancePerRotation(360);
  }

  /**
   * @return The absolute position of the MagEncoder in degrees
   */
  public double getAbsolutePosition() {
    return encoder.getDistance();
  }

  /**
   * @return The position of the MagEncoder shown as a value between 0 and 360
   */
  public double getPosition() {
    return getAbsolutePosition() % 360.0;
  }
}
