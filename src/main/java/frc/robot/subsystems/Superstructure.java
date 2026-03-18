package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GlobalConstants;
import frc.robot.simulation.BallLaunchHelper;
import frc.robot.simulation.GamePieceConstants;
import frc.robot.simulation.GamePieceSimulation;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.indexer.floor.Floor;
import frc.robot.subsystems.indexer.floor.FloorConstants;
import frc.robot.subsystems.indexer.tunnel.Tunnel;
import frc.robot.subsystems.indexer.tunnel.TunnelConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
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

  // Climber climber;

  @AutoLogOutput(key = "RobotStates/Shooting")
  public boolean shooting = false;

  @AutoLogOutput(key = "RobotStates/Tracking")
  public boolean tracking = false;

  @AutoLogOutput(key = "RobotStates/Intake In")
  public boolean intakeIn = false;

  public Trigger isShooting = new Trigger(() -> shooting);
  public Trigger isTracking = new Trigger(() -> tracking);
  public Trigger isSwallowing = new Trigger(() -> intakeIn && shooting);

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
      Tunnel tunnel) {
    this.drivebase = drivebase;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = shoot;
    this.floor = floor;
    this.intake = intake;
    this.tunnel = tunnel;
    // this.climber = climber;

    fuelSim = GamePieceSimulation.getInstance();

    // #region Triggers

    isShooting.onTrue(RunIndexer());
    isShooting.onFalse(StopIndexer());

    isTracking.onTrue(RunTracking());
    isTracking.onFalse(FixedShot());

    isShooting.whileTrue(
        Commands.repeatingSequence(
            Commands.runOnce(
                () -> {
                  if (RobotBase.isSimulation()) {
                    BallLaunchHelper.spawnWithLaunchCharacteristics(
                        fuelSim,
                        flywheel.getVelocity(),
                        hood.getPosition(),
                        -turret.getPosition(),
                        drivebase.getPose(),
                        drivebase.getVelocityRobotRelative());
                  }
                }),
            Commands.waitSeconds(4.0 / GlobalConstants.mainLoopFrequency)));

    isSwallowing.whileTrue(
        Commands.repeatingSequence(
            Commands.runOnce(() -> intake.changeSetpoint(ExtensionSetpoint.SHUFFLE_OUT)),
            Commands.waitSeconds(Extension.SHUFFLE_PERIOD),
            Commands.runOnce(() -> intake.changeSetpoint(ExtensionSetpoint.SHUFFLE_IN)),
            Commands.waitSeconds(Extension.SHUFFLE_PERIOD)));
  }

  Command RunIndexer() {
    return Commands.parallel(FloorForward(), TunnelForward());
  }

  Command StopIndexer() {
    return Commands.parallel(FloorOff(), TunnelOff());
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
    Rotation3d turretRotation = new Rotation3d(0, 0, -Units.degreesToRadians(turret.getPosition()));

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

  // #region State Toggles
  public Command ToggleShooting() {
    return Commands.runOnce(() -> shooting = !shooting);
  }

  public Command EnableShooting() {
    return Commands.runOnce(() -> shooting = true);
  }

  public Command DisableShooting() {
    return Commands.runOnce(() -> shooting = false);
  }

  public Command EnableTracking() {
    return Commands.runOnce(() -> tracking = true);
  }

  public Command DisableTracking() {
    return Commands.runOnce(() -> tracking = false);
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
  public Translation2d getTurretTarget(Pose2d robotPose) {
    robotPose = AllianceFlipUtil.apply(robotPose);
    if (robotPose.getX() < Units.inchesToMeters(150)) {
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
    return globalHeading
        .minus(drivebase.getPose().getRotation())
        .minus(Rotation2d.kCW_90deg)
        .times(-1);
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

  // #region Helper Sequences

  public Command HomeRobot() {
    return Commands.sequence(
        logMessage("Home Robot"),
        flywheel.changeSetpointC(0),
        tunnel.changeSetpoint(TunnelConstants.OFF),
        floor.changeSetpoint(FloorConstants.Off),
        DisableTracking(),
        DisableShooting(),
        intake.changeSetpoint(ExtensionSetpoint.RETRACTED_FAST),
        Commands.runOnce(() -> intakeIn = true),
        intake.changeSetpoint(Roller.Off),
        hood.changeSetpointC(0));
  }

  public Command ExtendIntake() {
    return Commands.sequence(
        logMessage("Fuel Intake"),
        intake.changeSetpoint(ExtensionSetpoint.EXTENDED_FAST),
        Commands.runOnce(() -> intakeIn = false),
        intake.changeSetpoint(Roller.FORWARD));
  }

  public Command RetractIntake() {
    return Commands.sequence(
        logMessage("Intake Retract"),
        intake.changeSetpoint(ExtensionSetpoint.RETRACTED_FAST),
        Commands.runOnce(() -> intakeIn = true));
  }

  public Command Dump() {
    return Commands.sequence(
        intake.changeSetpoint(Roller.FORWARD),
        intake.changeSetpoint(ExtensionSetpoint.EXTENDED_FAST),
        Commands.runOnce(() -> intakeIn = false),
        flywheel.changeSetpointC(-1000),
        TunnelReverse(),
        FloorReverse());
  }

  public Command ClimberDown() {
    return Commands.sequence(
        logMessage("Bring-Down"),
        intake.changeSetpoint(ExtensionSetpoint.RETRACTED_FAST),
        Commands.runOnce(() -> intakeIn = true),
        intake.changeSetpoint(0),
        turret.changeSetpoint(120),
        // climber.changeHookSetpoint(ClimberConstants.HOOK_SETPOINT, false)
        hood.changeSetpointC(10),
        flywheel.changeSetpointC(0));
  }

  // public Command ClimberUp() {
  //   return Commands.sequence(
  //       logMessage("Bring-Up"),
  //       climber
  //           .changeHookSetpoint(ClimberConstants.MAX_HEIGHT, false)
  //   );
  // }

  public Command TunnelForward() {
    return Commands.sequence(
        logMessage("Tunnel Forward"), tunnel.changeSetpoint(TunnelConstants.FORWARD));
  }

  public Command TunnelReverse() {
    return Commands.sequence(
        logMessage("Tunnel Reverse"), tunnel.changeSetpoint(TunnelConstants.REVERSE));
  }

  public Command TunnelOff() {
    return Commands.sequence(logMessage("Tunnel Off"), tunnel.changeSetpoint(TunnelConstants.OFF));
  }

  public Command FloorForward() {
    return Commands.sequence(
        logMessage("Floor Forward"), floor.changeSetpoint(FloorConstants.FORWARD));
  }

  public Command FloorReverse() {
    return Commands.sequence(
        logMessage("Floor Reverse"), floor.changeSetpoint(FloorConstants.REVERSE));
  }

  public Command FloorOff() {
    return Commands.sequence(logMessage("Floor Off"), floor.changeSetpoint(FloorConstants.Off));
  }

  public Command RunTracking() {
    return Commands.run(
        () -> {
          var calc = LaunchCalculator.getInstance();

          calc.setEstimatedPose(drivebase.getPose());
          calc.setFieldVelocity(drivebase.getVelocityFieldRelative());
          calc.clearLaunchingParameters();

          var params = calc.getParameters(getTurretTarget(drivebase.getPose()));

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
        flywheel);
  }

  public Command FixedShot() {
    return Commands.sequence(
        flywheel.changeSetpointC(2000), hood.changeSetpointC(10), turret.changeSetpoint(90));
  }

  // #region Autos

  public AutoRoutine TaxiShoot(AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("TaxiShoot");

    final AutoTrajectory Start = routine.trajectory("TaxiShoot", 0);

    Start.done().onTrue(EnableShooting());
    routine
        .active()
        .onTrue(Commands.sequence(Start.resetOdometry(), EnableTracking(), Start.cmd()));

    return routine;
  }

  public AutoRoutine OneCycle(AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("OneCycle");

    final AutoTrajectory path1 = routine.trajectory("OneCycle", 0);
    final AutoTrajectory path2 = routine.trajectory("OneCycle", 1);
    final AutoTrajectory path3 = routine.trajectory("OneCycle", 2);

    path1.atTimeBeforeEnd(1).onTrue(ExtendIntake());
    path3.atTimeBeforeEnd(0.5).onTrue(EnableTracking());
    path3
        .done()
        .onTrue(EnableShooting().andThen(intake.changeSetpoint(ExtensionSetpoint.RETRACTED_SLOW)));

    routine
        .active()
        .onTrue(Commands.sequence(path1.resetOdometry(), path1.cmd(), path2.cmd(), path3.cmd()));

    return routine;
  }

  public AutoRoutine OneCycleDepot(AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("OneCycleDepot");

    final AutoTrajectory path1 = routine.trajectory("OneCycleDepot", 0);
    final AutoTrajectory path2 = routine.trajectory("OneCycleDepot", 1);
    final AutoTrajectory path3 = routine.trajectory("OneCycleDepot", 2);

    path1.atTimeBeforeEnd(1).onTrue(ExtendIntake());
    path3.atTimeBeforeEnd(0.5).onTrue(EnableTracking());
    path3
        .done()
        .onTrue(EnableShooting().andThen(intake.changeSetpoint(ExtensionSetpoint.RETRACTED_SLOW)));

    routine
        .active()
        .onTrue(Commands.sequence(path1.resetOdometry(), path1.cmd(), path2.cmd(), path3.cmd()));

    return routine;
  }

  // public AutoRoutine ClimbOnly(AutoFactory factory) {
  //   final AutoRoutine routine = factory.newRoutine("ClimbOnly");

  //   final AutoTrajectory Start = routine.trajectory("ClimbOnly", 0);

  //   Start.done().onTrue(firstRungAutoClimb(2));
  //   routine.active().onTrue(Commands.sequence(Start.resetOdometry(), Start.cmd()));

  //   return routine;
  // }

  public AutoRoutine GreedyAuto(AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("GreedyAuto");

    final AutoTrajectory start = routine.trajectory("Start", 0);
    final AutoTrajectory crossover1 = routine.trajectory("crossover1", 1);
    final AutoTrajectory intake = routine.trajectory("intake", 2);
    final AutoTrajectory crossover2 = routine.trajectory("crossover2", 3);
    final AutoTrajectory end = routine.trajectory("end", 4);

    start
        .atTimeBeforeEnd(0.4)
        .onTrue(
            Commands.sequence(
                EnableShooting(),
                Commands.waitSeconds(2.5),
                new ScheduleCommand(crossover1.cmd()),
                DisableTracking()));

    crossover1.done().onTrue(Commands.parallel(ExtendIntake(), new ScheduleCommand(intake.cmd())));

    intake.done().onTrue(Commands.parallel(RetractIntake(), new ScheduleCommand(crossover2.cmd())));

    crossover2.done().onTrue(Commands.parallel(EnableShooting(), new ScheduleCommand(end.cmd())));

    return routine;
  }

  // public AutoRoutine FireClimb(AutoFactory factory) {
  //   final AutoRoutine routine = factory.newRoutine("Fire Climb");

  //   final AutoTrajectory start = routine.trajectory("Start",0);
  //   final AutoTrajectory shoot = routine.trajectory("Shoot", 1);
  //   final AutoTrajectory climb = routine.trajectory("Climb",2);

  //   start.atTimeBeforeEnd(0.3).onTrue(Commands.sequence(EnableShooting(),
  // Commands.waitSeconds(8), new ScheduleCommand(shoot.cmd()), DisableTracking()));

  //   shoot.done().onTrue(Commands.parallel(firstRungAutoClimb(2), new
  // ScheduleCommand(climb.cmd())));

  //   return routine;
  // }
}
