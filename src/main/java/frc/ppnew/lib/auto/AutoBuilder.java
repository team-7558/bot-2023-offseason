package frc.ppnew.lib.auto;

import frc.ppnew.lib.commands.*;
import frc.ppnew.lib.path.PathConstraints;
import frc.ppnew.lib.path.PathPlannerPath;
import frc.ppnew.lib.util.HolonomicPathFollowerConfig;
import frc.ppnew.lib.util.ReplanningConfig;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/** Utility class used to build auto routines */
public class AutoBuilder {
  private static boolean configured = false;

  private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
  private static Supplier<Pose2d> getPose;
  private static Consumer<Pose2d> resetPose;

  // Pathfinding builders
  private static boolean pathfindingConfigured = false;
  private static QuadFunction<Pose2d, PathConstraints, Double, Double, Command>
      pathfindToPoseCommandBuilder;
  private static TriFunction<PathPlannerPath, PathConstraints, Double, Command>
      pathfindThenFollowPathCommandBuilder;

  /**
   * Configures the AutoBuilder for a holonomic drivetrain.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
   *     speeds
   * @param robotRelativeOutput a consumer for setting the robot's field-relative chassis speeds
   * @param config {@link com.pathplanner.lib.util.HolonomicPathFollowerConfig} for configuring the
   *     path following commands
   * @param driveSubsystem the subsystem for the robot's drive
   * @throws AutoBuilderException if AutoBuilder has already been configured
   */
  public static void configureHolonomic(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> robotRelativeOutput,
      HolonomicPathFollowerConfig config,
      Subsystem driveSubsystem) {
    if (configured) {
      throw new AutoBuilderException(
          "Auto builder has already been configured. Please only configure auto builder once");
    }

    AutoBuilder.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathHolonomic(
                path,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
                driveSubsystem);
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;

    AutoBuilder.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel, rotationDelayDistance) ->
            new PathfindHolonomic(
                pose,
                constraints,
                goalEndVel,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
                rotationDelayDistance,
                driveSubsystem);
    AutoBuilder.pathfindThenFollowPathCommandBuilder =
        (path, constraints, rotationDelayDistance) ->
            new PathfindThenFollowPathHolonomic(
                path,
                constraints,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
                rotationDelayDistance,
                driveSubsystem);
    AutoBuilder.pathfindingConfigured = true;
  }

  /**
   * Configures the AutoBuilder for a differential drivetrain using a RAMSETE path follower.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param speedsSupplier a supplier for the robot's current chassis speeds
   * @param output a consumer for setting the robot's chassis speeds
   * @param replanningConfig Path replanning configuration
   * @param driveSubsystem the subsystem for the robot's drive
   * @throws AutoBuilderException if AutoBuilder has already been configured
   */
  public static void configureRamsete(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> speedsSupplier,
      Consumer<ChassisSpeeds> output,
      ReplanningConfig replanningConfig,
      Subsystem driveSubsystem) {
    if (configured) {
      throw new AutoBuilderException(
          "Auto builder has already been configured. Please only configure auto builder once");
    }

    AutoBuilder.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathRamsete(
                path, poseSupplier, speedsSupplier, output, replanningConfig, driveSubsystem);
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;

    AutoBuilder.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel, rotationDelayDistance) ->
            new PathfindRamsete(
                pose.getTranslation(),
                constraints,
                goalEndVel,
                poseSupplier,
                speedsSupplier,
                output,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindThenFollowPathCommandBuilder =
        (path, constraints, rotationDelayDistance) ->
            new PathfindThenFollowPathRamsete(
                path,
                constraints,
                poseSupplier,
                speedsSupplier,
                output,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindingConfigured = true;
  }

  /**
   * Configures the AutoBuilder for a differential drivetrain using a RAMSETE path follower.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param speedsSupplier a supplier for the robot's current chassis speeds
   * @param output a consumer for setting the robot's chassis speeds
   * @param b Tuning parameter (b &gt; 0 rad^2/m^2) for which larger values make convergence more
   *     aggressive like a proportional term.
   * @param zeta Tuning parameter (0 rad^-1 &lt; zeta &lt; 1 rad^-1) for which larger values provide
   *     more damping in response.
   * @param replanningConfig Path replanning configuration
   * @param driveSubsystem the subsystem for the robot's drive
   * @throws AutoBuilderException if AutoBuilder has already been configured
   */
  public static void configureRamsete(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> speedsSupplier,
      Consumer<ChassisSpeeds> output,
      double b,
      double zeta,
      ReplanningConfig replanningConfig,
      Subsystem driveSubsystem) {
    if (configured) {
      throw new AutoBuilderException(
          "Auto builder has already been configured. Please only configure auto builder once");
    }

    AutoBuilder.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathRamsete(
                path,
                poseSupplier,
                speedsSupplier,
                output,
                b,
                zeta,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;

    AutoBuilder.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel, rotationDelayDistance) ->
            new PathfindRamsete(
                pose.getTranslation(),
                constraints,
                goalEndVel,
                poseSupplier,
                speedsSupplier,
                output,
                b,
                zeta,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindThenFollowPathCommandBuilder =
        (path, constraints, rotationDelayDistance) ->
            new PathfindThenFollowPathRamsete(
                path,
                constraints,
                poseSupplier,
                speedsSupplier,
                output,
                b,
                zeta,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindingConfigured = true;
  }

  /**
   * Configures the AutoBuilder for a differential drivetrain using a LTVUnicycleController path
   * follower.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param speedsSupplier a supplier for the robot's current chassis speeds
   * @param output a consumer for setting the robot's chassis speeds
   * @param dt Period of the robot control loop in seconds (default 0.02)
   * @param replanningConfig Path replanning configuration
   * @param driveSubsystem the subsystem for the robot's drive
   * @throws AutoBuilderException if AutoBuilder has already been configured
   */
  public static void configureLTV(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> speedsSupplier,
      Consumer<ChassisSpeeds> output,
      double dt,
      ReplanningConfig replanningConfig,
      Subsystem driveSubsystem) {
    if (configured) {
      throw new AutoBuilderException(
          "Auto builder has already been configured. Please only configure auto builder once");
    }

    AutoBuilder.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathLTV(
                path, poseSupplier, speedsSupplier, output, dt, replanningConfig, driveSubsystem);
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;

    AutoBuilder.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel, rotationDelayDistance) ->
            new PathfindLTV(
                pose.getTranslation(),
                constraints,
                goalEndVel,
                poseSupplier,
                speedsSupplier,
                output,
                dt,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindThenFollowPathCommandBuilder =
        (path, constraints, rotationDelayDistance) ->
            new PathfindThenFollowPathLTV(
                path,
                constraints,
                poseSupplier,
                speedsSupplier,
                output,
                dt,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindingConfigured = true;
  }

  /**
   * Configures the AutoBuilder for a differential drivetrain using a LTVUnicycleController path
   * follower.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param speedsSupplier a supplier for the robot's current chassis speeds
   * @param output a consumer for setting the robot's chassis speeds
   * @param qelems The maximum desired error tolerance for each state.
   * @param relems The maximum desired control effort for each input.
   * @param dt Period of the robot control loop in seconds (default 0.02)
   * @param replanningConfig Path replanning configuration
   * @param driveSubsystem the subsystem for the robot's drive
   * @throws AutoBuilderException if AutoBuilder has already been configured
   */
  public static void configureLTV(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> speedsSupplier,
      Consumer<ChassisSpeeds> output,
      Vector<N3> qelems,
      Vector<N2> relems,
      double dt,
      ReplanningConfig replanningConfig,
      Subsystem driveSubsystem) {
    if (configured) {
      throw new AutoBuilderException(
          "Auto builder has already been configured. Please only configure auto builder once");
    }

    AutoBuilder.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathLTV(
                path,
                poseSupplier,
                speedsSupplier,
                output,
                qelems,
                relems,
                dt,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;

    AutoBuilder.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel, rotationDelayDistance) ->
            new PathfindLTV(
                pose.getTranslation(),
                constraints,
                goalEndVel,
                poseSupplier,
                speedsSupplier,
                output,
                qelems,
                relems,
                dt,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindThenFollowPathCommandBuilder =
        (path, constraints, rotationDelayDistance) ->
            new PathfindThenFollowPathLTV(
                path,
                constraints,
                poseSupplier,
                speedsSupplier,
                output,
                qelems,
                relems,
                dt,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindingConfigured = true;
  }

  /**
   * Configures the AutoBuilder with custom path following command builder. Building pathfinding
   * commands is not supported if using a custom command builder.
   *
   * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @throws AutoBuilderException if AutoBuilder has already been configured
   */
  public static void configureCustom(
      Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose) {
    if (configured) {
      throw new AutoBuilderException(
          "Auto builder has already been configured. Please only configure auto builder once");
    }

    AutoBuilder.pathFollowingCommandBuilder = pathFollowingCommandBuilder;
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;

    AutoBuilder.pathfindingConfigured = false;
  }

  /**
   * Returns whether the AutoBuilder has been configured.
   *
   * @return true if the AutoBuilder has been configured, false otherwise
   */
  public static boolean isConfigured() {
    return configured;
  }

  /**
   * Returns whether the AutoBuilder has been configured for pathfinding.
   *
   * @return true if the AutoBuilder has been configured for pathfinding, false otherwise
   */
  public static boolean isPathfindingConfigured() {
    return pathfindingConfigured;
  }

  /**
   * Builds a command to follow a path with event markers.
   *
   * @param path the path to follow
   * @return a path following command with events for the given path
   * @throws AutoBuilderException if the AutoBuilder has not been configured
   */
  public static Command followPathWithEvents(PathPlannerPath path) {
    if (!isConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a path following command before being configured");
    }

    return new FollowPathWithEvents(pathFollowingCommandBuilder.apply(path), path, getPose);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @param rotationDelayDistance The distance the robot should move from the start position before
   *     attempting to rotate to the final rotation
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose,
      PathConstraints constraints,
      double goalEndVelocity,
      double rotationDelayDistance) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return pathfindToPoseCommandBuilder.apply(
        pose, constraints, goalEndVelocity, rotationDelayDistance);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
    return pathfindToPose(pose, constraints, goalEndVelocity, 0);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
    return pathfindToPose(pose, constraints, 0);
  }

  /**
   * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
   * drivetrain, the pose rotation delay distance will have no effect.
   *
   * @param goalPath The path to pathfind to, then follow
   * @param pathfindingConstraints The constraints to use while pathfinding
   * @param rotationDelayDistance The distance the robot should move from the start position before
   *     attempting to rotate to the final rotation
   * @return A command to pathfind to a given path, then follow the path
   */
  public static Command pathfindThenFollowPath(
      PathPlannerPath goalPath,
      PathConstraints pathfindingConstraints,
      double rotationDelayDistance) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return pathfindThenFollowPathCommandBuilder.apply(
        goalPath, pathfindingConstraints, rotationDelayDistance);
  }

  /**
   * Build a command to pathfind to a given path, then follow that path.
   *
   * @param goalPath The path to pathfind to, then follow
   * @param pathfindingConstraints The constraints to use while pathfinding
   * @return A command to pathfind to a given path, then follow the path
   */
  public static Command pathfindThenFollowPath(
      PathPlannerPath goalPath, PathConstraints pathfindingConstraints) {
    return pathfindThenFollowPath(goalPath, pathfindingConstraints, 0);
  }

  /**
   * Get the starting pose from its JSON representation. This is only used internally.
   *
   * @param startingPoseJson JSON object representing a starting pose.
   * @return The Pose2d starting pose
   */
  public static Pose2d getStartingPoseFromJson(JSONObject startingPoseJson) {
    JSONObject pos = (JSONObject) startingPoseJson.get("position");
    double x = ((Number) pos.get("x")).doubleValue();
    double y = ((Number) pos.get("y")).doubleValue();
    double deg = ((Number) startingPoseJson.get("rotation")).doubleValue();

    return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
  }

  /**
   * Builds an auto command for the given auto name.
   *
   * @param autoName the name of the auto to build
   * @return an auto command for the given auto name
   */
  public static Command buildAuto(String autoName) {
    try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
      return getAutoCommandFromJson(json);
    } catch (AutoBuilderException e) {
      throw e;
    } catch (Exception e) {
      throw new RuntimeException(e.getMessage());
    }
  }

  /**
   * Builds an auto command from the given JSON object.
   *
   * @param autoJson the JSON object to build the command from
   * @return an auto command built from the JSON object
   */
  public static Command getAutoCommandFromJson(JSONObject autoJson) {
    JSONObject commandJson = (JSONObject) autoJson.get("command");

    Command autoCommand = CommandUtil.commandFromJson(commandJson);
    if (autoJson.get("startingPose") != null) {
      Pose2d startPose = getStartingPoseFromJson((JSONObject) autoJson.get("startingPose"));
      return Commands.sequence(Commands.runOnce(() -> resetPose.accept(startPose)), autoCommand);
    } else {
      return autoCommand;
    }
  }

  /** Functional interface for a function that takes 3 inputs */
  @FunctionalInterface
  public interface TriFunction<In1, In2, In3, Out> {
    /**
     * Apply the inputs to this function
     *
     * @param in1 Input 1
     * @param in2 Input 2
     * @param in3 Input 3
     * @return Output
     */
    Out apply(In1 in1, In2 in2, In3 in3);
  }

  /** Functional interface for a function that takes 4 inputs */
  @FunctionalInterface
  public interface QuadFunction<In1, In2, In3, In4, Out> {
    /**
     * Apply the inputs to this function
     *
     * @param in1 Input 1
     * @param in2 Input 2
     * @param in3 Input 3
     * @param in4 Input 4
     * @return Output
     */
    Out apply(In1 in1, In2 in2, In3 in3, In4 in4);
  }
}
