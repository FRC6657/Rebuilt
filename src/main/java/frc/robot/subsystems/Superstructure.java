package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
// import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GlobalConstants;
import frc.robot.simulation.BallLaunchHelper;
import frc.robot.simulation.GamePieceConstants;
import frc.robot.simulation.GamePieceSimulation;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.indexer.floor.Floor;
import frc.robot.subsystems.indexer.floor.FloorConstants;
import frc.robot.subsystems.indexer.tunnel.Tunnel;
import frc.robot.subsystems.indexer.tunnel.TunnelConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;
import frc.robot.subsystems.intake.IntakeConstants.Roller;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.LaunchCalculator;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates all robot subsystems into unified high-level commands. Acts as the central command
 * factory for multi-subsystem actions.
 */
public class Superstructure {

  // Subsystems
  Drivebase drivebase;
  Turret turret;
  Hood hood;
  Flywheel flywheel;
  Floor floor;
  Intake intake;
  Tunnel tunnel;
  Climber climber;

  @AutoLogOutput(key = "RobotStates/isShooting")
  public boolean isShooting = false;

  @AutoLogOutput(key = "RobotStates/isTracking")
  public boolean isTracking = false;

  public Trigger shootingEnabled = new Trigger(() -> isShooting);
  public Trigger trackingEnabled = new Trigger(() -> isTracking);

  private final GamePieceSimulation fuelSim;

  /**
   * The field-relative position the turret aims at (blue alliance tower center). Keep this variable
   * looking at Blue Alliance targets.
   */
  Translation2d turretTarget =
      new Translation2d(
          GamePieceConstants.BLUE_TOWER_CENTER.getX(), GamePieceConstants.BLUE_TOWER_CENTER.getY());

  /** Constructs the Superstructure with references to all robot subsystems. */
  public Superstructure(
      Drivebase drivebase,
      Turret turret,
      Hood hood,
      Flywheel shoot,
      Intake intake,
      Floor floor,
      Tunnel tunnel,
      Climber climber) {
    this.drivebase = drivebase;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = shoot;
    this.floor = floor;
    this.intake = intake;
    this.tunnel = tunnel;
    this.climber = climber;

    fuelSim = GamePieceSimulation.getInstance();

    shootingEnabled.onTrue(
        Commands.parallel(startShooting(), Commands.runOnce(() -> isTracking = true)));
    shootingEnabled.onFalse(stopShooting());

    trackingEnabled.whileTrue(softTracking());
    trackingEnabled.onFalse(turret.restingSetpoint());

    shootingEnabled.whileTrue(
        Commands.repeatingSequence(
            Commands.runOnce(
                () -> {
                  BallLaunchHelper.spawnWithLaunchCharacteristics(
                      fuelSim,
                      flywheel.getVelocity(),
                      hood.getPosition(),
                      turret.getPosition(),
                      drivebase.getPose(),
                      drivebase.getVelocityRobotRelative());
                }),
            Commands.waitSeconds(4.0 / GlobalConstants.mainLoopFrequency)));
  }

  Command startShooting() {
    return Commands.parallel(FloorOn(), tunnelForward());
  }

  Command stopShooting() {
    return Commands.parallel(
        Commands.sequence(FloorBackward(), Commands.waitSeconds(1.5), FloorOff()),
        Commands.sequence(tunnelBackward(), Commands.waitSeconds(0.75), tunnelOff()));
  }

  /**
   * Returns the 3D poses of the turret, hood, and intake for AdvantageScope visualization. Computes
   * each component's pose relative to the robot origin using current mechanism positions.
   *
   * @return array of [turret pose, hood pose, intake pose]
   */
  @AutoLogOutput(key = "3DComponents")
  public Pose3d[] get3DComponents() {

    // Compute turret yaw rotation from current turret position
    Rotation3d turretRotation = new Rotation3d(0, 0, Units.degreesToRadians(turret.getPosition()));

    // Rotate the hood offset by turret yaw to get the hood's position in robot frame
    Translation3d rotatedHoodOffset = TurretConstants.HOOD_OFFSET.rotateBy(turretRotation);
    Translation3d hoodPosition = TurretConstants.TURRET_CENTER.plus(rotatedHoodOffset);

    // Hood pitch is local, then rotated into turret frame
    Rotation3d hoodLocalPitch = new Rotation3d(Units.degreesToRadians(hood.getPosition()), 0, 0);
    Rotation3d hoodRotation = hoodLocalPitch.rotateBy(turretRotation);

    return new Pose3d[] {
      new Pose3d(TurretConstants.TURRET_CENTER, turretRotation),
      new Pose3d(hoodPosition, hoodRotation),
      // Intake position projected along its mounting angle (0.2229 rad from horizontal)
      new Pose3d(
          -Math.cos(0.222900) * Units.inchesToMeters(intake.getPosition()),
          0,
          -Math.sin(0.222900) * Units.inchesToMeters(intake.getPosition()),
          new Rotation3d())
    };
  }

  public Command toggleShooting() {
    return Commands.runOnce(() -> isShooting = !isShooting);
  }

  public Command shootingOn() {
    return Commands.runOnce(() -> isShooting = true);
  }

  public Command shootingOff() {
    return Commands.runOnce(() -> isShooting = false);
  }

  public Command trackingOn() {
    return Commands.runOnce(() -> isTracking = true);
  }

  public Command trackingOff() {
    return Commands.parallel(Commands.runOnce(() -> isTracking = false), shootingOff());
  }

  /**
   * Calculates the turret's position in field coordinates by rotating the turret offset into the
   * field frame and adding it to the robot pose.
   *
   * @return the turret's field-relative position
   */
  public Translation2d getTurretGlobalPosition() {
    Translation2d offset =
        new Translation2d(
            TurretConstants.TURRET_CENTER.getX(), TurretConstants.TURRET_CENTER.getY());
    return drivebase
        .getPose()
        .getTranslation()
        .plus(offset.rotateBy(drivebase.getPose().getRotation()));
  }

  @AutoLogOutput(key = "TurretTarget")
  public Translation2d turretTarget(Pose2d robotPose) {
    robotPose = AllianceFlipUtil.apply(robotPose);
    if (robotPose.getX() < Units.inchesToMeters(150)) { // TODO: find proper values
      return new Translation2d(
          GamePieceConstants.BLUE_TOWER_CENTER.getX(), GamePieceConstants.BLUE_TOWER_CENTER.getY());
    } else {
      if (robotPose.getY() > Units.inchesToMeters(150)) {
        return new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(225));
      } else {
        return new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(75));
      }
    }
  }

  /**
   * Calculates the field-relative heading from the turret to a goal position using atan2.
   *
   * @param goalPose the target position in field coordinates
   * @return the angle from the turret to the goal
   */
  public Rotation2d getGlobalTargetHeading(Translation2d goalPose) {
    return Rotation2d.fromRadians(
        Math.atan2(
            (goalPose.getY() - getTurretGlobalPosition().getY()),
            (goalPose.getX() - getTurretGlobalPosition().getX())));
  }

  /**
   * Converts a field-relative heading into a turret-relative heading by subtracting the robot's
   * current heading and a 90-degree CW offset.
   *
   * @param globalHeading the field-relative target heading
   * @return the turret-relative heading in degrees
   */
  public Rotation2d getRelativeTurretHeading(Rotation2d globalHeading) {
    return globalHeading.minus(drivebase.getPose().getRotation()).minus(Rotation2d.kCW_90deg);
  }

  /** Schedules a turret tracking command that aims the turret at the turret target. */
  public void runTurretTest() {
    Rotation2d targetAngle = getGlobalTargetHeading(turretTarget);
    CommandScheduler.getInstance()
        .schedule(turret.changeSetpoint(() -> getRelativeTurretHeading(targetAngle).getDegrees()));
  }

  /**
   * Creates a command that logs a message to AdvantageKit.
   *
   * @param message the message to log
   * @return a command that logs the message once
   */
  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }

  // public Command shoot() {
  //  return Commands.sequence(logMessage("Shoot"), flywheel.changeSetpointC(12));
  // }

  public Command HomeRobot() {
    return Commands.sequence(
        logMessage("Home Robot"),
        flywheel.changeSetpointC(0),
        tunnel.changeSetpoint(TunnelConstants.OFF),
        floor.changeSetpoint(FloorConstants.Off),
        turret.changeSetpoint(0),
        intake.changeSetpoint(ExtensionSetpoint.RETRACTED_FAST),
        intake.changeSetpoint(Roller.Off),
        hood.changeSetpointC(0));
  }

  public Command intakeFuel() {
    return Commands.sequence(
        logMessage("Fuel Intake"),
        intake.changeSetpoint(ExtensionSetpoint.EXTENDED_FAST),
        Commands.waitSeconds(0.5),
        intake.changeSetpoint(Roller.FORWARD));
  }

  public Command intakeRetract() {
    return Commands.sequence(
        logMessage("Intake Retract"),
        intake.changeSetpoint(ExtensionSetpoint.RETRACTED_FAST),
        Commands.waitSeconds(0.5),
        intake.changeSetpoint(ExtensionSetpoint.Off),
        intake.changeSetpoint(Roller.Off));
  }

  public Command driveInClimber() {
    return Commands.sequence(
        logMessage("Drive-in"),
        climber.changeSetpoints(
            ClimberConstants.DRIVEIN_SETPOINT, ClimberConstants.Pedal.PEDAL_MIN_ANGLE));
  }

  public Command bringDownClimber() {
    return Commands.sequence(
        logMessage("Bring-Down"),
        climber.changeSetpoints(
            ClimberConstants.LOW_SETPOINT, ClimberConstants.Pedal.PEDAL_MAX_ANGLE));
  }

  public Command bringUpClimber() {
    return Commands.sequence(
        logMessage("Bring-Up"),
        climber.changeSetpoints(
            ClimberConstants.HOOK_SETPOINT, ClimberConstants.Pedal.PEDAL_MAX_ANGLE));
  }

  public Command fullClimb() {
    return Commands.sequence(
        logMessage("Full Climb"),
        driveInClimber(),
        Commands.waitSeconds(2.0),
        bringDownClimber(),
        Commands.waitSeconds(3.0),
        bringUpClimber(),
        Commands.waitSeconds(3.0),
        bringDownClimber(),
        Commands.waitSeconds(3.0),
        bringUpClimber(),
        Commands.waitSeconds(3.0),
        bringDownClimber());
  }

  public Command firstRungAutoClimb(double prepSeconds) {
    return Commands.sequence(
        logMessage("First-rung climb auto"),
        trackingOff(),
        driveInClimber(),
        Commands.waitSeconds(prepSeconds),
        Commands.race(
            drivebase.driveTeleop(() -> new ChassisSpeeds(0.2, 0.0, 0.0), () -> false),
            Commands.waitSeconds(0.5)),
        bringDownClimber());
  }

  public Command tunnelLaunch() {
    return Commands.sequence(
        logMessage("Tunnel Launch"), tunnel.changeSetpoint(TunnelConstants.FORWARD));
  }

  public Command tunnelForward() {
    return Commands.sequence(
        logMessage("Tunnel Off"), tunnel.changeSetpoint(TunnelConstants.FORWARD));
  }

  public Command tunnelOff() {
    return Commands.sequence(logMessage("Tunnel Off"), tunnel.changeSetpoint(TunnelConstants.OFF));
  }

  public Command tunnelBackward() {
    return Commands.sequence(
        logMessage("Tunnel Off"), tunnel.changeSetpoint(TunnelConstants.REVERSE));
  }

  public Command FloorOn() {
    return Commands.sequence(logMessage("Floor On"), floor.changeSetpoint(FloorConstants.FORWARD));
  }

  public Command FloorBackward() {
    return Commands.sequence(logMessage("Floor On"), floor.changeSetpoint(FloorConstants.REVERSE));
  }

  public Command FloorOff() {
    return Commands.sequence(logMessage("Floor On"), floor.changeSetpoint(FloorConstants.FORWARD));
  }

  // public Command flywheelShoot() {
  //   return Commands.sequence(logMessage("Flywheel Shoot"), flywheel.changeSetpointC(60));
  // }

  // public Command flywheelOff() {
  //   return Commands.sequence(logMessage("Flywheel Off"), flywheel.changeSetpointC(0));
  // }

  public Command softTracking() {
    return Commands.run(
        () -> {
          // isShooting = true;
          var calc = LaunchCalculator.getInstance();

          calc.setEstimatedPose(drivebase.getPose());
          calc.setFieldVelocity(drivebase.getVelocityFieldRelative());
          calc.clearLaunchingParameters();

          var params = calc.getParameters(turretTarget(drivebase.getPose()));

          flywheel.changeSetpoint(
              Units.radiansPerSecondToRotationsPerMinute(params.flywheelSpeed()));
          hood.changeSetpoint(Math.toDegrees(params.hoodAngle()));
          Rotation2d turretHeading = getRelativeTurretHeading(params.turretAngle());
          double omega = drivebase.getVelocityFieldRelative().omegaRadiansPerSecond;
          double feedforward = Math.toDegrees(params.turretVelocity() - omega);
          turret.changeSetpoint(turretHeading.getDegrees(), feedforward);
        },
        turret,
        hood,
        flywheel)
    // .finallyDo(() -> isShooting = false);
    ;
  }

  public Command tempSetTrackingOn() {
    return Commands.run(() -> isTracking = true);
  }

  public Command tempSetTrackingOff() {
    return Commands.run(() -> isTracking = false);
  }

  public AutoRoutine TaxiShoot(AutoFactory factory, boolean mirror) {
    final AutoRoutine routine = factory.newRoutine("TaxiShoot");

    String mirrorFlag = mirror ? "mirrored_" : "";

    final AutoTrajectory Start = routine.trajectory(mirrorFlag + "TaxiShoot", 0);

    Start.done().onTrue(shootingOn());
    routine.active().onTrue(Commands.sequence(Start.resetOdometry(), Start.cmd()));

    return routine;
  }
}
