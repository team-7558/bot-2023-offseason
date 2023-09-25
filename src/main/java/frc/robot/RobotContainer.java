// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIONavX;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team7558.limelightVision.Limelight;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Mode;
import frc.robot.commands.LimelightFollow;
import frc.robot.commands.LimelightToggle;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.MK4IRobotConfig;
import frc.robot.configs.SierraRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private Limelight m_limelight = new Limelight();
  private Drivetrain m_drivetrain;
  private Intake m_intake = new Intake();
  private boolean limelight_state = false;

  private PneumaticHub hub = new PneumaticHub();
  private Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();
  private final Map<String, Command> autoEventMap = new HashMap<>();

  private boolean driveCalibrated;

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */

    hub.enableCompressorAnalog(100, 120);

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_DEFAULT:
        case ROBOT_2023_MK4I:
        case ROBOT_2022_SIERRA:
          {
            // create the specific RobotConfig subclass instance first
            if (Constants.getRobot() == Constants.RobotType.ROBOT_2023_MK4I) {
              config = new MK4IRobotConfig();
            } else if (Constants.getRobot() == Constants.RobotType.ROBOT_2022_SIERRA) {
              config = new SierraRobotConfig();
            } else {
              config = new DefaultRobotConfig();
            }

            GyroIO gyro = new GyroIONavX();

            int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
            int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
            int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
            double[] steerOffsets = config.getSwerveSteerOffsets();
            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        driveMotorCANIDs[0],
                        steerMotorCANDIDs[0],
                        steerEncoderCANDIDs[0],
                        steerOffsets[0]),
                    0,
                    config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        driveMotorCANIDs[1],
                        steerMotorCANDIDs[1],
                        steerEncoderCANDIDs[1],
                        steerOffsets[1]),
                    1,
                    config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        driveMotorCANIDs[2],
                        steerMotorCANDIDs[2],
                        steerEncoderCANDIDs[2],
                        steerOffsets[2]),
                    2,
                    config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        driveMotorCANIDs[3],
                        steerMotorCANDIDs[3],
                        steerEncoderCANDIDs[3],
                        steerOffsets[3]),
                    3,
                    config.getRobotMaxVelocity());

            m_drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            // new Pneumatics(new PneumaticsIORev());
            // new Vision(new VisionIOPhotonVision(config.getCameraName()));
            break;
          }
        case ROBOT_SIMBOT:
          {
            config = new MK4IRobotConfig();
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, config.getRobotMaxVelocity());
            m_drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            // new Pneumatics(new PneumaticsIO() {});
            // AprilTagFieldLayout layout;
            // try {
            //   layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            // } catch (IOException e) {
            //   layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            // }
            // new Vision(
            //     new VisionIOSim(
            //         layout,
            //         m_drivetrain::getPose,
            //         RobotConfig.getInstance().getRobotToCameraTransform()));

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, config.getRobotMaxVelocity());

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, config.getRobotMaxVelocity());

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, config.getRobotMaxVelocity());

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, config.getRobotMaxVelocity());
      m_drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      // new Pneumatics(new PneumaticsIORev() {});
      // new Vision(new VisionIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    configureAutoCommands();

    driveCalibrated = false;
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    m_drivetrain.setDefaultCommand(
        new TeleopSwerve(
            m_drivetrain,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            () -> oi.getDriverRightTrigger().getAsBoolean() ? 0.25 : 1.00));

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle


    oi.getOperatorLeftBumper().whileTrue(new LimelightToggle(m_limelight));

    oi.getOperatorA().whileTrue(new RunIntake(m_intake));

    oi.getDriverLeftTrigger().whileTrue(new LimelightFollow(m_limelight, m_drivetrain));

    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(m_drivetrain::disableFieldRelative, m_drivetrain),
                Commands.runOnce(m_drivetrain::enableFieldRelative, m_drivetrain),
                m_drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(m_drivetrain::zeroGyroscope, m_drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(m_drivetrain::enableXstance, m_drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(m_drivetrain::disableXstance, m_drivetrain));



  }

  /** Use this method to define your commands for autonomous mode. */
  public void configureAutoCommands() {

  }

  /**
   * Manually calibrates the drivetrain angle motor offsets. Calibration only happens the first time
   * this method is called.
   *
   * @return {@code true} if the drive was calibrated, {@code false} if the calibration was already
   *     done.
   */
  public boolean calibrateDrivetrain() {
    if (!driveCalibrated) {
      m_drivetrain.calibrateDrive();
      driveCalibrated = true;
      return true;
    }
    return false;
  }

  public Command readyDrive() {
    return Commands.runOnce(m_drivetrain::disableXstance, m_drivetrain);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
