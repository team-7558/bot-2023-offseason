package frc.lib.team3061.swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePositionDeg", drivePositionDeg);
    table.put("DriveDistanceMeters", driveDistanceMeters);
    table.put("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
    table.put("DriveAppliedPercentage", driveAppliedPercentage);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("DriveTempCelsius", driveTempCelsius);
    table.put("AngleAbsolutePositionDeg", angleAbsolutePositionDeg);
    table.put("AnglePositionDeg", anglePositionDeg);
    table.put("AngleVelocityRevPerMin", angleVelocityRevPerMin);
    table.put("AngleAppliedPercentage", angleAppliedPercentage);
    table.put("AngleCurrentAmps", angleCurrentAmps);
    table.put("AngleTempCelsius", angleTempCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionDeg = table.getDouble("DrivePositionDeg", drivePositionDeg);
    driveDistanceMeters = table.getDouble("DriveDistanceMeters", driveDistanceMeters);
    driveVelocityMetersPerSec = table.getDouble("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
    driveAppliedPercentage = table.getDouble("DriveAppliedPercentage", driveAppliedPercentage);
    driveCurrentAmps = table.getDoubleArray("DriveCurrentAmps", driveCurrentAmps);
    driveTempCelsius = table.getDoubleArray("DriveTempCelsius", driveTempCelsius);
    angleAbsolutePositionDeg = table.getDouble("AngleAbsolutePositionDeg", angleAbsolutePositionDeg);
    anglePositionDeg = table.getDouble("AnglePositionDeg", anglePositionDeg);
    angleVelocityRevPerMin = table.getDouble("AngleVelocityRevPerMin", angleVelocityRevPerMin);
    angleAppliedPercentage = table.getDouble("AngleAppliedPercentage", angleAppliedPercentage);
    angleCurrentAmps = table.getDoubleArray("AngleCurrentAmps", angleCurrentAmps);
    angleTempCelsius = table.getDoubleArray("AngleTempCelsius", angleTempCelsius);
  }

  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.drivePositionDeg = this.drivePositionDeg;
    copy.driveDistanceMeters = this.driveDistanceMeters;
    copy.driveVelocityMetersPerSec = this.driveVelocityMetersPerSec;
    copy.driveAppliedPercentage = this.driveAppliedPercentage;
    copy.driveCurrentAmps = this.driveCurrentAmps.clone();
    copy.driveTempCelsius = this.driveTempCelsius.clone();
    copy.angleAbsolutePositionDeg = this.angleAbsolutePositionDeg;
    copy.anglePositionDeg = this.anglePositionDeg;
    copy.angleVelocityRevPerMin = this.angleVelocityRevPerMin;
    copy.angleAppliedPercentage = this.angleAppliedPercentage;
    copy.angleCurrentAmps = this.angleCurrentAmps.clone();
    copy.angleTempCelsius = this.angleTempCelsius.clone();
    return copy;
  }
}
