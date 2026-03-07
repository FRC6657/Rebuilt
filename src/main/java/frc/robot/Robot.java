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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.vision.ApriltagCameraIO;
import frc.robot.subsystems.vision.ApriltagCameraIO_Real;
import frc.robot.subsystems.vision.ApriltagCameraIO_Sim;
import frc.robot.subsystems.vision.ApriltagCameras;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.CommandKeypad;
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
  private CommandKeypad operator = new CommandKeypad(1);

  private final Drivebase drivebase;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Floor floor;
  private final Tunnel tunnel;
  private final Climber climber;
  private final Superstructure superstructure;

  private final ApriltagCameras cameras;

  private final AutoFactory autoFactory;

  //private double testHoodAngle = 10;
  //private double testFlywheelRPM = 2000;

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

    cameras =
        new ApriltagCameras(
            drivebase::addVisionMeasurement,
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO[] {
                  new ApriltagCameraIO_Real(VisionConstants.Black1),
                  new ApriltagCameraIO_Real(VisionConstants.Black2),
                  new ApriltagCameraIO_Real(VisionConstants.White1),
                  new ApriltagCameraIO_Real(VisionConstants.White2),
                }
                : new ApriltagCameraIO[] {
                  new ApriltagCameraIO_Sim(VisionConstants.Black1, drivebase::getPose),
                  new ApriltagCameraIO_Sim(VisionConstants.Black2, drivebase::getPose),
                  new ApriltagCameraIO_Sim(VisionConstants.White1, drivebase::getPose),
                  new ApriltagCameraIO_Sim(VisionConstants.White2, drivebase::getPose),
                });

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

    // #region Autos
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("TaxiShoot", superstructure.TaxiShoot(autoFactory).cmd());
    autoChooser.addOption("OneCycle", superstructure.OneCycle(autoFactory).cmd());
    autoChooser.addOption("OneCycleDepot", superstructure.OneCycleDepot(autoFactory).cmd());
  }

  public static boolean replay = false;

  @Override
  public void robotInit() {

    Pathfinding.setPathfinder(new LocalADStarAK());

    Logger.recordMetadata("Arborbotics", "PortH");

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

    // #region Driver Controls

    drivebase.setDefaultCommand(
        drivebase.driveTeleop(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.1)
                        * DrivebaseConstants.kMaxLinearSpeed
                        * (superstructure.shooting ? 0.2 : 0.5),
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.1)
                        * DrivebaseConstants.kMaxLinearSpeed
                        * (superstructure.shooting ? 0.2 : 0.5),
                    -MathUtil.applyDeadband(driver.getRightX(), 0.1)
                        * DrivebaseConstants.kMaxAngularSpeed
                        * (superstructure.shooting ? 0.2 : 0.375)),
            () -> superstructure.shooting));

    driver.rightTrigger().onTrue(superstructure.EnableShooting());
    driver.rightTrigger().onFalse(superstructure.DisableShooting());
    driver.leftTrigger().onTrue(superstructure.ToggleShooting());

    operator.m3().onTrue(superstructure.ToggleShooting());
    operator.plus().onTrue(superstructure.EnableTracking());
    operator.enter().onTrue(superstructure.DisableTracking());

    // driver.x().onTrue(superstructure.HomeRobot());
    // operator.knob_press().onTrue(superstructure.HomeRobot());

    operator.circle().onTrue(superstructure.ExtendIntake());
    operator.triangle().onTrue(superstructure.RetractIntake());

    // operator.m1().onTrue(superstructure.ClimberUp());
    // operator.m2().onTrue(superstructure.ClimberDown());

    operator
        .numClr()
        .onTrue(climber.changeHookSetpoint(1, true))
        .onFalse(climber.changeHookSetpoint(0, true));
    operator
        .num7()
        .onTrue(climber.changeHookSetpoint(-1, true))
        .onFalse(climber.changeHookSetpoint(0, true));

    // #region Debug Controls

    // RPM and Hood Trim
    // operator
    //     .knob_left()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               if (operator.knob_press().getAsBoolean()) {
    //                 testHoodAngle -= 0.25;
    //               } else {
    //                 testFlywheelRPM -= 50;
    //               }
    //             }));
    // operator
    //     .knob_right()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               if (operator.knob_press().getAsBoolean()) {
    //                 testHoodAngle += 0.25;
    //               } else {
    //                 testFlywheelRPM += 50;
    //               }
    //             }));

    // // Enable Dial Controled Flywheel And Hood
    // driver
    //     .a()
    //     .toggleOnTrue(
    //         Commands.repeatingSequence(
    //             Commands.runOnce(() -> hood.changeSetpoint(testHoodAngle)),
    //             Commands.runOnce(() -> flywheel.changeSetpoint(testFlywheelRPM))))
    //     .toggleOnFalse(
    //         Commands.sequence(
    //             hood.changeSetpointC(10),
    //             flywheel.changeSetpointC(0),
    //             tunnel.changeSetpoint(0),
    //             floor.changeSetpoint(0)));

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (RobotBase.isSimulation()) {
      GamePieceSimulation.getInstance().update();
    }
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
    // // Force State
    // CommandScheduler.getInstance()
    //     .schedule(
    //         Commands.sequence(
    //             superstructure.DisableTracking(),
    //             superstructure.EnableTracking(),
    //             superstructure.EnableShooting(),
    //             superstructure.DisableShooting()));
  }
}
