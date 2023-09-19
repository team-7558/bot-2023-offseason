/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import edu.wpi.first.wpilibj.SPI;
import frc.lib.NavX.AHRS;

public class GyroIONavX implements GyroIO {
  private final AHRS gyro;

  public GyroIONavX() {
    gyro = new AHRS(SPI.Port.kMXP, (byte) 50);
    gyro.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    // inputs.positionDeg = -gyro.getYaw(); // degrees
    double angle = gyro.getRotation2d().getDegrees() * 1.0124; // )%360 - 180;
    inputs.positionDeg = ((angle) + (((int) Math.abs(angle) / 360) + 1) * 360) % 360 - 180;
    // inputs.positionDeg = (gyro.getRotation2d().getDegrees()*1.0124)%360 - 180;
    // System.out.println("*********************GYRO ROT2D: " + gyro.getRotation2d().getDegrees());
    inputs.tiltDeg =
        Math.signum(gyro.getRoll())
            * Math.sqrt(
                (gyro.getRoll() * gyro.getRoll())
                    + (gyro.getPitch() * gyro.getPitch())); // gyro.getRoll();
    inputs.velocityDegPerSec = -gyro.getRate(); // degrees per second
  }
}
