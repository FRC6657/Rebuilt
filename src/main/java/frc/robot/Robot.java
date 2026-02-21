// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalConstants.opButtons;
import frc.robot.simulation.GamePieceSimulation;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO_Real;
import frc.robot.subsystems.climber.ClimberIO_Sim;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.drivebase.DrivebaseConstants;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_Redux;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.indexer.floor.Floor;
import frc.robot.subsystems.indexer.floor.FloorIO_Real;
import frc.robot.subsystems.indexer.floor.FloorIO_Sim;
import frc.robot.subsystems.indexer.tunnel.Tunnel;
import frc.robot.subsystems.indexer.tunnel.TunnelIO_Real;
import frc.robot.subsystems.indexer.tunnel.TunnelIO_Sim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO_Real;
import frc.robot.subsystems.intake.IntakeIO_Sim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO_Real;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO_Sim;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO_Real;
import frc.robot.subsystems.shooter.hood.HoodIO_Sim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO_Real;
import frc.robot.subsystems.shooter.turret.TurretIO_Sim;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

@SuppressWarnings("resource")
public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandGenericHID operator =
      new CommandGenericHID(1); // arduino buttons 1 to 16, left to right, then top to bottom

  private final Drivebase drivebase;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Floor floor;
  private final Tunnel tunnel;
  private final Climber climber;
  private final Superstructure superstructure;

  // private final ApriltagCameras cameras;

  private final AutoFactory autoFactory;

  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  public Robot() {

    DriverStation.silenceJoystickConnectionWarning(true);

    drivebase =
        new Drivebase(
            RobotBase.isReal() ? new GyroIO_Redux() : new GyroIO() {},
            RobotBase.isReal()
                ? new ModuleIO[] {
                  new ModuleIO_Real(DrivebaseConstants.kFrontLeftModuleConstants),
                  new ModuleIO_Real(DrivebaseConstants.kFrontRightModuleConstants),
                  new ModuleIO_Real(DrivebaseConstants.kBackLeftModuleConstants),
                  new ModuleIO_Real(DrivebaseConstants.kBackRightModuleConstants)
                }
                : new ModuleIO[] {
                  new ModuleIO_Sim(DrivebaseConstants.kFrontLeftModuleConstants),
                  new ModuleIO_Sim(DrivebaseConstants.kFrontRightModuleConstants),
                  new ModuleIO_Sim(DrivebaseConstants.kBackLeftModuleConstants),
                  new ModuleIO_Sim(DrivebaseConstants.kBackRightModuleConstants)
                });

    turret = new Turret(RobotBase.isReal() ? new TurretIO_Real() : new TurretIO_Sim());
    hood = new Hood(RobotBase.isReal() ? new HoodIO_Real() : new HoodIO_Sim());
    flywheel = new Flywheel(RobotBase.isReal() ? new FlywheelIO_Real() : new FlywheelIO_Sim());
    intake = new Intake(RobotBase.isReal() ? new IntakeIO_Real() : new IntakeIO_Sim());
    floor = new Floor(RobotBase.isReal() ? new FloorIO_Real() : new FloorIO_Sim());
    tunnel = new Tunnel(RobotBase.isReal() ? new TunnelIO_Real() : new TunnelIO_Sim());
    climber = new Climber(RobotBase.isReal() ? new ClimberIO_Real() : new ClimberIO_Sim());

    superstructure =
        new Superstructure(drivebase, turret, hood, flywheel, intake, floor, tunnel, climber);

    // cameras =
    //     new ApriltagCameras(
    //         drivebase::addVisionMeasurement,
    //         RobotBase.isReal() || replay
    //             ? new ApriltagCameraIO[] {
    //               new ApriltagCameraIO_Real(VisionConstants.Black1),
    //               new ApriltagCameraIO_Real(VisionConstants.Black2),
    //               new ApriltagCameraIO_Real(VisionConstants.White1),
    //               new ApriltagCameraIO_Real(VisionConstants.White2),
    //             }
    //             : new ApriltagCameraIO[] {
    //               new ApriltagCameraIO_Sim(VisionConstants.Black1, drivebase::getPose),
    //               new ApriltagCameraIO_Sim(VisionConstants.Black2, drivebase::getPose),
    //               new ApriltagCameraIO_Sim(VisionConstants.White1, drivebase::getPose),
    //               new ApriltagCameraIO_Sim(VisionConstants.White2, drivebase::getPose),
    //             });

    autoFactory =
        new AutoFactory(
            drivebase::getPose, drivebase::resetPose, drivebase::followTrajectory, true, drivebase);

    AutoBuilder.configure(
        drivebase::getPose,
        drivebase::resetPose,
        drivebase::getVelocityRobotRelative,
        (speeds, feedforwards) -> drivebase.drive(speeds),
        DrivebaseConstants.kPathPlannerPID,
        DrivebaseConstants.kPathPlannerConfig,
        () -> false,
        drivebase);

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("TaxiShoot", superstructure.TaxiShoot(autoFactory, false).cmd());
    autoChooser.addOption("ClimbOnly", superstructure.firstRungAutoClimb(0));
  }

  public static boolean replay = false;

  @Override
  public void robotInit() {

    Pathfinding.setPathfinder(new LocalADStarAK());

    Logger.recordMetadata("Arborbotics", "RobotName");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      if (!replay) {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      } else {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
      }
    }

    drivebase.setDefaultCommand(
        drivebase.driveTeleop(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.1)
                        * DrivebaseConstants.kMaxLinearSpeed
                        * 0.5,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.1)
                        * DrivebaseConstants.kMaxLinearSpeed
                        * 0.5,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.1)
                        * DrivebaseConstants.kMaxAngularSpeed
                        * 0.375),
            () -> superstructure.isShooting));

    // -- Test Bindings --
    /*
    driver.a().onTrue(intake.changeSetpoint(ExtensionSetpoint.EXTENDED_FAST));
    driver.a().onFalse(intake.changeSetpoint(ExtensionSetpoint.RETRACTED_FAST));
    driver.b().onTrue(intake.changeSetpoint(ExtensionSetpoint.EXTENDED_SLOW));
    driver.b().onFalse(intake.changeSetpoint(ExtensionSetpoint.RETRACTED_SLOW));
    driver.x().onTrue(intake.changeSetpoint(RollerSetpoint.FORWARD));
    driver.x().onFalse(intake.changeSetpoint(RollerSetpoint.Off));
    driver.y().onTrue(flywheel.changeSetpointC(3000));
    driver.y().onFalse(flywheel.changeSetpointC(0));
    driver.leftBumper().onTrue(floor.changeSetpoint(FloorSetpoint.FORWARD));
    driver.leftBumper().onFalse(floor.changeSetpoint(FloorSetpoint.Off));
    driver.rightBumper().onTrue(tunnel.changeSetpoint(TunnelSetpoint.FORWARD));
    driver.rightBumper().onFalse(tunnel.changeSetpoint(TunnelSetpoint.Off));
    driver.leftTrigger().onTrue(hood.changeSetpointC(40));
    driver.leftTrigger().onFalse(hood.changeSetpointC(10));
    */

    // driver
    //     .rightTrigger()
    //     .onTrue(
    //         Commands.repeatingSequence(
    //                 Commands.runOnce(
    //                     () -> {
    //                       BallLaunchHelper.spawnWithLaunchCharacteristics(
    //                           fuelSim,
    //                           flywheel.getVelocity(),
    //                           hood.getPosition(),
    //                           turret.getPosition(),
    //                           drivebase.getPose(),
    //                           drivebase.getVelocityRobotRelative());
    //                     }),
    //                 Commands.waitSeconds(4 / GlobalConstants.mainLoopFrequency))
    //             .until(() -> !driver.rightTrigger().getAsBoolean()));

    // driver.leftTrigger().whileTrue(superstructure.softTracking());

    driver.leftTrigger().onTrue(superstructure.shootingOn());
    driver.leftTrigger().onFalse(superstructure.shootingOff());
    driver.y().onTrue(superstructure.toggleShooting());
    operator.button(16).onTrue(superstructure.toggleShooting());

    operator.button(15).onTrue(superstructure.trackingOn());
    operator.button(14).onTrue(superstructure.trackingOff());

    driver.x().onTrue(superstructure.HomeRobot());
    operator.button(opButtons.HomeRobot.id).onTrue(superstructure.HomeRobot());

    operator.button(opButtons.Intake.id).onTrue(superstructure.intakeFuel());
    operator.button(opButtons.StopIntake.id).onTrue(superstructure.intakeRetract());

    operator.button(opButtons.FullClimb.id).whileTrue(superstructure.fullClimb());
    operator
        .button(opButtons.FullClimb.id)
        .onFalse(superstructure.logMessage("fullClimb Sequence Aborted"));
    operator.button(opButtons.ManualClimberInit.id).onTrue(superstructure.driveInClimber());
    operator.button(opButtons.ManualClimberDown.id).onTrue(superstructure.bringDownClimber());
    operator.button(opButtons.ManualClimberUp.id).onTrue(superstructure.bringUpClimber());

    // operator.button(1).whileTrue(superstructure.tempSetTrackingOn());
    // operator.button(2).whileTrue(superstructure.tempSetTrackingOff());

    Logger.start();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    GamePieceSimulation.getInstance().update();
    // Logger.recordOutput(
    //     "Black1Transform",
    //     new Pose3d(drivebase.getPose()).transformBy(VisionConstants.Black1.robotToCamera));
    // Logger.recordOutput(
    //     "Black2Transform",
    //     new Pose3d(drivebase.getPose()).transformBy(VisionConstants.Black2.robotToCamera));
    // Logger.recordOutput(
    //     "White1Transform",
    //     new Pose3d(drivebase.getPose()).transformBy(VisionConstants.White1.robotToCamera));
    // Logger.recordOutput(
    //     "White2Transform",
    //     new Pose3d(drivebase.getPose()).transformBy(VisionConstants.White2.robotToCamera));

    // Publishes all code from the SmartDashboard to the keypad screen
    SmartDashboard.putBoolean("RobotEnabled", isEnabled());
    SmartDashboard.putBoolean("Autonomous", isAutonomous());
    SmartDashboard.putBoolean("Teleop", isTeleop());
    SmartDashboard.putBoolean("Test", isTest());
    SmartDashboard.putBoolean("Disabled", isDisabled());
    SmartDashboard.putBoolean("isShooting", superstructure.isShooting);
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("isTracking", superstructure.isTracking);

    // Helpful combined string
    String mode = "PASSIVE";
    if (isEnabled()) {
      if (isAutonomous()) {
        mode = "AUTO   ";
      } else if (isTeleop()) {
        mode = "TELEOP ";
      } else if (isTest()) {
        mode = "TEST   ";
      }
    }
    SmartDashboard.putString("RobotMode", mode);
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(autoChooser.get());
  }

  @Override
  public void teleopInit() {
    if (autoChooser.get() != null) {
      autoChooser.get().cancel();
    }
    superstructure.shootingOff();
  }
}
