// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import frc.ppnew.lib.auto.AutoBuilder;
import frc.ppnew.lib.path.PathConstraints;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Mode;
import frc.robot.commands.LimelightFollow;
import frc.robot.commands.LimelightToggle;
import frc.robot.commands.LimelightTrack;
import frc.robot.commands.LogLimelightValues;
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
  private Intake m_intake;
  private RobotConfig config;
  private Limelight m_limelight = new Limelight();
  private Drivetrain m_drivetrain;


  private PneumaticHub hub = new PneumaticHub();
  private Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();
  private final Map<String, Command> autoEventMap = new HashMap<>();


  private Pose2d nodes[] = new Pose2d[9];



  private boolean driveCalibrated;

  public RobotConfig getConfig() {
    return this.config;
  }


  private void configPathPlanner() {


    nodes[0] = new Pose2d(new Translation2d(2, 0.52),new Rotation2d(Units.degreesToRadians(180)));
    nodes[1] = new Pose2d(new Translation2d(2, 1.09),new Rotation2d(Units.degreesToRadians(180)));
    nodes[2] = new Pose2d(new Translation2d(2, 1.64),new Rotation2d(Units.degreesToRadians(180)));
    nodes[3] = new Pose2d(new Translation2d(2, 2.19),new Rotation2d(Units.degreesToRadians(180)));
    nodes[4] = new Pose2d(new Translation2d(2, 2.76),new Rotation2d(Units.degreesToRadians(180)));
    nodes[5] = new Pose2d(new Translation2d(2, 3.33),new Rotation2d(Units.degreesToRadians(180)));
    nodes[6] = new Pose2d(new Translation2d(2, 3.89),new Rotation2d(Units.degreesToRadians(180)));
    nodes[7] = new Pose2d(new Translation2d(2, 4.43),new Rotation2d(Units.degreesToRadians(180)));
    nodes[8] = new Pose2d(new Translation2d(2, 5.0),new Rotation2d(Units.degreesToRadians(180)));

  }

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configPathPlanner();
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */


    // hub.enableCompressorAnalog(100, 120);
    // this.m_intake = new Intake();
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

    m_drivetrain.resetOdometry(new Pose2d(5.56,0.73,Rotation2d.fromDegrees(90)));

    oi.getDriverCircle().whileTrue(new LogLimelightValues(m_limelight));
    
    oi.getOperatorLeftBumper().whileTrue(new LimelightToggle(m_limelight));

    // oi.getOperatorA().whileTrue(new RunIntake(m_intake));

    // oi.getDriverLeftTrigger().whileTrue(new LimelightFollow(m_limelight, m_drivetrain));

    // oi.getDriverLeftTrigger().whileTrue(new LimelightTrack(m_drivetrain, m_limelight));

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

    PathConstraints constraints = new PathConstraints(1.0,
    0.5,
    Units.degreesToRadians(RobotConfig.getInstance().getRobotMaxAngularVelocity() * 0.25),
    Units.degreesToRadians(RobotConfig.getInstance().getRobotMaxAngularVelocity() * 0.1)
  );

    oi.getOperatorA().onTrue(AutoBuilder.pathfindToPose(
      nodes[0],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorX().onTrue(AutoBuilder.pathfindToPose(
      nodes[1],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorY().onTrue(AutoBuilder.pathfindToPose(
      nodes[2],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorB().onTrue(AutoBuilder.pathfindToPose(
      nodes[3],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorRightBumper().onTrue(AutoBuilder.pathfindToPose(
      nodes[4],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorRightTrigger().onTrue(AutoBuilder.pathfindToPose(
      nodes[5],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorLeftBumper().onTrue(AutoBuilder.pathfindToPose(
      nodes[6],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorLeftTrigger().onTrue(AutoBuilder.pathfindToPose(
      nodes[7],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

  oi.getOperatorDPad(90).onTrue(AutoBuilder.pathfindToPose(
      nodes[8],
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ));

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
