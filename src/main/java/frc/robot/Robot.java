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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.simulation.GamePieceSimulation;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.drivebase.DrivebaseConstants;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_CTRE;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.vision.ApriltagCameraIO_Real;
import frc.robot.subsystems.vision.ApriltagCameraIO_Sim;
import frc.robot.subsystems.vision.ApriltagCameras;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

@SuppressWarnings({"unused", "resource"})
public class Robot extends LoggedRobot {

  // Controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandGenericHID operator = new CommandGenericHID(1);

  // Subsystems
  private final ApriltagCameras cameras;
  private final Drivebase drivebase;
  private final Turret turret;
  private final Hood hood;
  private final Shooter shoot;
  private final Intake intake;

  // Superstructure
  private final Superstructure superstructure;

  // Auto Stuff
  private final AutoFactory autoFactory;
  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  public Robot() {

    // Get rid  of annoying warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);

    drivebase =
        new Drivebase(
            RobotBase.isReal() ? new GyroIO_CTRE() : new GyroIO() {},
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

    cameras =
        new ApriltagCameras(
            drivebase::addVisionMeasurement,
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.ExampleCameraInfo1)
                : new ApriltagCameraIO_Sim(VisionConstants.ExampleCameraInfo1, drivebase::getPose),
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.ExampleCameraInfo2)
                : new ApriltagCameraIO_Sim(VisionConstants.ExampleCameraInfo1, drivebase::getPose));

    turret = new Turret(new TurretIO() {});

    hood = new Hood(new HoodIO() {});

    shoot = new Shooter(new ShooterIO() {});

    superstructure = new Superstructure(drivebase, turret, hood, shoot);

    intake = new Intake(new IntakeIO() {});

    AutoBuilder.configure(
        drivebase::getPose,
        drivebase::resetPose,
        drivebase::getVelocityRobotRelative,
        (speeds, feedforwards) -> drivebase.drive(speeds),
        DrivebaseConstants.kPathPlannerPID,
        DrivebaseConstants.kPathPlannerConfig,
        () -> false,
        drivebase);

    autoFactory =
        new AutoFactory(
            drivebase::getPose, drivebase::resetPose, drivebase::followTrajectory, true, drivebase);

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
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
                        * 0.375)));

    autoChooser.addOption(
        "TestAuto", Commands.run(() -> drivebase.drive(new ChassisSpeeds(1, 0, 0))));
    
    driver.a().onTrue(Superstructure.intakeFuel()).onFalse(Superstructure.HomeRobot());
    
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    GamePieceSimulation.getInstance().update();
  }

  @Override
  public void teleopPeriodic() {
    superstructure.runTurretTest();
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
  }
}
