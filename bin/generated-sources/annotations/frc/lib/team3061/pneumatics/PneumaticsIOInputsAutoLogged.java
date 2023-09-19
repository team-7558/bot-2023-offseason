package frc.lib.team3061.pneumatics;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PneumaticsIOInputsAutoLogged extends PneumaticsIO.PneumaticsIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("HighPressurePSI", highPressurePSI);
    table.put("LowPressurePSI", lowPressurePSI);
    table.put("CompressorActive", compressorActive);
    table.put("CompressorCurrentAmps", compressorCurrentAmps);
    table.put("FlowLPM", flowLPM);
    table.put("VolumeL", volumeL);
  }

  @Override
  public void fromLog(LogTable table) {
    highPressurePSI = table.getDouble("HighPressurePSI", highPressurePSI);
    lowPressurePSI = table.getDouble("LowPressurePSI", lowPressurePSI);
    compressorActive = table.getBoolean("CompressorActive", compressorActive);
    compressorCurrentAmps = table.getDouble("CompressorCurrentAmps", compressorCurrentAmps);
    flowLPM = table.getDouble("FlowLPM", flowLPM);
    volumeL = table.getDouble("VolumeL", volumeL);
  }

  public PneumaticsIOInputsAutoLogged clone() {
    PneumaticsIOInputsAutoLogged copy = new PneumaticsIOInputsAutoLogged();
    copy.highPressurePSI = this.highPressurePSI;
    copy.lowPressurePSI = this.lowPressurePSI;
    copy.compressorActive = this.compressorActive;
    copy.compressorCurrentAmps = this.compressorCurrentAmps;
    copy.flowLPM = this.flowLPM;
    copy.volumeL = this.volumeL;
    return copy;
  }
}
