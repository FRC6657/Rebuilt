package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.drivebase.DrivebaseConstants.ModuleConstants;
import java.util.Queue;

/**
 * Real hardware implementation of a swerve module using TalonFX drive/turn motors and a Canandmag
 * absolute encoder. Registers drive/turn positions for high-frequency odometry sampling.
 */
public class ModuleIO_Real implements ModuleIO {

  private final ModuleConstants constants;

  // Module hardware
  private final TalonFX drive;
  private final TalonFX turn;
  private final Canandmag encoder;

  // Useful Status Signals
  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveAppliedVolts;
  private final BaseStatusSignal driveCurrent;
  private final BaseStatusSignal driveSupplyCurrent;

  private final BaseStatusSignal turnPosition;
  private final BaseStatusSignal turnVelocity;
  private final BaseStatusSignal turnAppliedVolts;
  private final BaseStatusSignal turnCurrent;

  // Control Signals
  private final VelocityVoltage drivePID = new VelocityVoltage(0.0);
  private final VoltageOut driveOpenLoop = new VoltageOut(0);
  private final PositionVoltage turnPID = new PositionVoltage(0.0);

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIO_Real(ModuleConstants constants) {

    this.constants = constants;

    // Assign Hardware
    drive = new TalonFX(constants.driveID());
    turn = new TalonFX(constants.turnID());
    encoder = new Canandmag(constants.encoderID());

    // Configure Motors
    drive.getConfigurator().apply(DrivebaseConstants.driveConfig);
    turn.getConfigurator().apply(DrivebaseConstants.turnConfig);

    // Assign Status Signals
    drivePosition = drive.getPosition();
    driveVelocity = drive.getVelocity();
    driveAppliedVolts = drive.getMotorVoltage();
    driveCurrent = drive.getStatorCurrent();
    driveSupplyCurrent = drive.getSupplyCurrent();

    turnPosition = turn.getPosition();
    turnVelocity = turn.getVelocity();
    turnAppliedVolts = turn.getMotorVoltage();
    turnCurrent = turn.getStatorCurrent();

    encoder.setSettings(new CanandmagSettings().setInvertDirection(true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        DrivebaseConstants.kOdometryFrequency, drivePosition, turnPosition);

    // Set Status Signal Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    // Optimize Bus Utilization
    drive.optimizeBusUtilization();
    turn.optimizeBusUtilization();

    // if (constants.encoderID() == 12) {
    //   encoder.setSettings(
    //       new CanandmagSettings().setZeroOffset(Units.radiansToRotations(1.4733885467641583)));
    // } else if (constants.encoderID() == 11) {
    //   encoder.setSettings(
    //       new CanandmagSettings().setZeroOffset(Units.radiansToRotations(4.6644520807632635)));
    // } else if (constants.encoderID() == 10) {
    //   encoder.setSettings(
    //       new CanandmagSettings().setZeroOffset(Units.radiansToRotations(4.6383744073692075)));
    // } else if (constants.encoderID() == 9) {
    //   encoder.setSettings(
    //       new CanandmagSettings().setZeroOffset(Units.radiansToRotations(1.564660403643354)));
    // }

    encoder.setSettings(new CanandmagSettings().setInvertDirection(true));

    // Seed relative encoder
    turn.setPosition(encoder.getAbsPosition());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drive.getPosition());
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turn.getPosition());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Refresh Status Signals
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    // Update Inputs
    inputs.name = constants.name();

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveOutputVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(encoder.getAbsPosition());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositions =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void changeDriveSetpoint(double metersPerSecond, boolean openLoop) {
    // If the robot is stopped, set the drive to 0 volts
    if (metersPerSecond == 0 && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      drive.setControl(new VoltageOut(0));
    } else { // Otherwise, set the drive to the desired velocity
      drive.setControl(
          openLoop
              ? driveOpenLoop.withOutput(
                  RobotController.getBatteryVoltage()
                      * (metersPerSecond * DrivebaseConstants.kDriveRotorToMeters)
                      / (Units.radiansPerSecondToRotationsPerMinute(
                              DrivebaseConstants.kDriveMotor.freeSpeedRadPerSec)
                          / 60d))
              : drivePID.withVelocity(metersPerSecond));
    }
  }

  @Override
  public void changeTurnSetpoint(Rotation2d rotation) {
    // Set the module rotation to the desired position
    turn.setControl(turnPID.withPosition(rotation.getRotations()));
  }

  @Override
  /** Reset the drive encoder to 0 */
  public void resetDriveEncoder() {
    drive.setPosition(0);
  }
}
