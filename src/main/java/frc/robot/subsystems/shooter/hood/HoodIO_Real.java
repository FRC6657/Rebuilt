package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;

/**
 * Real hardware implementation of the hood using a TalonFX with on-board position PID. The setpoint
 * is clamped between min and max angles and converted from degrees to rotations.
 */
public class HoodIO_Real implements HoodIO {

  private TalonFX hoodMotor = new TalonFX(GlobalConstants.CAN.Hood.id);
  // Position control request initialized to the home position (in rotations)
  private PositionVoltage positionVoltage =
      new PositionVoltage(HoodConstants.INITIAL_SETPOINT / HoodConstants.CONVERSION_FACTOR);

  public HoodIO_Real() {

    hoodMotor.getConfigurator().apply(HoodConstants.CONFIG);

    var position = hoodMotor.getPosition();
    var temp = hoodMotor.getDeviceTemp();
    var voltage = hoodMotor.getMotorVoltage();
    var statorCurrent = hoodMotor.getStatorCurrent();

    position.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    temp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4d);
    voltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    statorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    hoodMotor.optimizeBusUtilization();

    hoodMotor.setPosition(HoodConstants.INITIAL_SETPOINT / HoodConstants.CONVERSION_FACTOR);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    inputs.temp = hoodMotor.getDeviceTemp().getValueAsDouble();
    inputs.statorCurrent = hoodMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.position = hoodMotor.getPosition().getValueAsDouble() * HoodConstants.CONVERSION_FACTOR;

    hoodMotor.setControl(positionVoltage);
  }

  @Override
  public void changeSetpoint(double setpoint) {
    // Clamp to safe range then convert degrees to rotations for the motor controller
    positionVoltage.Position =
        MathUtil.clamp(setpoint, HoodConstants.MIN_SETPOINT, HoodConstants.MAX_SETPOINT)
            / HoodConstants.CONVERSION_FACTOR;
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(
        positionVoltage.Position * HoodConstants.CONVERSION_FACTOR,
        hoodMotor.getPosition().getValueAsDouble() * HoodConstants.CONVERSION_FACTOR,
        HoodConstants.POSITION_TOLERANCE);
  }
}
