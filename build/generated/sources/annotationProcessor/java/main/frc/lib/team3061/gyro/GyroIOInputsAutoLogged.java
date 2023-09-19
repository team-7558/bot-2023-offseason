package frc.lib.team3061.gyro;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("PositionDeg", positionDeg);
    table.put("TiltDeg", tiltDeg);
    table.put("VelocityDegPerSec", velocityDegPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.getBoolean("Connected", connected);
    positionDeg = table.getDouble("PositionDeg", positionDeg);
    tiltDeg = table.getDouble("TiltDeg", tiltDeg);
    velocityDegPerSec = table.getDouble("VelocityDegPerSec", velocityDegPerSec);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.positionDeg = this.positionDeg;
    copy.tiltDeg = this.tiltDeg;
    copy.velocityDegPerSec = this.velocityDegPerSec;
    return copy;
  }
}
