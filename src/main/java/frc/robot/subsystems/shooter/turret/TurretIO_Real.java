package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;

/**
 * Real hardware implementation of the turret using a TalonFX with MotionMagic position control.
 * Input angles are wrapped to 0-360 degrees before being converted to rotations.
 */
public class TurretIO_Real implements TurretIO {

  private double position = 0.0;
  private double setpoint = 0.0;
  private TalonFX turretMotor = new TalonFX(GlobalConstants.CAN.Turret.id);
  private PositionVoltage positionVoltage =
      new PositionVoltage(TurretConstants.ORIGIN_POSITION / TurretConstants.CONVERSION_FACTOR);

  public TurretIO_Real() {

    turretMotor.getConfigurator().apply(TurretConstants.CONFIG);

    var position = turretMotor.getPosition();
    var velocity = turretMotor.getVelocity();
    var acceleration = turretMotor.getAcceleration();
    var temp = turretMotor.getDeviceTemp();
    var voltage = turretMotor.getMotorVoltage();
    var statorCurrent = turretMotor.getStatorCurrent();

    position.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    velocity.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    acceleration.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    temp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4d);
    voltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    statorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    turretMotor.optimizeBusUtilization();

    changeSetpoint(0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // turretMotor.setControl(positionVoltage);

    inputs.temp = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.statorCurrent = turretMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = turretMotor.getMotorVoltage().getValueAsDouble();
    position = turretMotor.getPosition().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR;
    inputs.position = position;
    inputs.velocity =
        turretMotor.getVelocity().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR;
    inputs.acceleration =
        turretMotor.getAcceleration().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR;
    inputs.setpoint = setpoint;
  }

  @Override
  public void changeSetpoint(double newSetpoint) {
    // Wrap the input angle to [0, 360) degrees
    double clampedInput = newSetpoint % 360;
    if (clampedInput < 0) {
      clampedInput = clampedInput + 360;
    }

    setpoint = clampedInput;

    if (Math.abs(clampedInput - (position - TurretConstants.ORIGIN_POSITION)) > 180) {
      /*if the desired position is that far away from the current position, then we want to check if we can go the other way! */
      if (clampedInput + 360 < TurretConstants.ROTATION_RANGE) {
        clampedInput += 360;
      }
    }
    // Add initial offset and convert degrees to motor rotations
    positionVoltage.Position =
        (clampedInput + TurretConstants.ORIGIN_POSITION) / TurretConstants.CONVERSION_FACTOR;
    positionVoltage.FeedForward = 0;
  }

  @Override
  public void changeSetpoint(double setpoint, double feedforwardDegPerSec) {
    changeSetpoint(setpoint);
    positionVoltage.FeedForward = feedforwardDegPerSec * TurretConstants.FF_VOLTS_PER_DEG_SEC;
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(
        positionVoltage.Position * TurretConstants.CONVERSION_FACTOR,
        turretMotor.getPosition().getValueAsDouble() * TurretConstants.CONVERSION_FACTOR,
        TurretConstants.POSITION_TOLERANCE);
  }
}
