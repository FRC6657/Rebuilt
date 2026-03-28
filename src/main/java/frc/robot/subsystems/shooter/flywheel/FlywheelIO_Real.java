package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;

/**
 * Real hardware implementation of the flywheel using two TalonFX motors in leader/follower
 * configuration with velocity PID control.
 */
public class FlywheelIO_Real implements FlywheelIO {

  private TalonFX leaderMotor = new TalonFX(GlobalConstants.CAN.Shooter_Leader.id);
  private TalonFX followerMotor = new TalonFX(GlobalConstants.CAN.Shooter_Follower.id);

  private VelocityVoltage setpoint = new VelocityVoltage(0);

  public FlywheelIO_Real() {

    leaderMotor.getConfigurator().apply(FlywheelConstants.CONFIG);
    followerMotor.getConfigurator().apply(FlywheelConstants.CONFIG);

    var velocity = leaderMotor.getVelocity();
    var acceleration = leaderMotor.getAcceleration();

    var leaderTemp = leaderMotor.getDeviceTemp();
    var leaderVoltage = leaderMotor.getMotorVoltage();
    var leaderStatorCurrent = leaderMotor.getSupplyCurrent();

    var followerTemp = followerMotor.getDeviceTemp();
    var followerVoltage = followerMotor.getMotorVoltage();
    var followerStatorCurrent = followerMotor.getSupplyCurrent();

    velocity.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    acceleration.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    leaderTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    followerTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    leaderVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    followerVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    leaderStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    followerStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    if (setpoint.Velocity == 0) {
      leaderMotor.setControl(new VoltageOut(0));
    } else {
      leaderMotor.setControl(setpoint);
    }

    followerMotor.setControl(
        new Follower(GlobalConstants.CAN.Shooter_Leader.id, MotorAlignmentValue.Opposed));

    inputs.velocity = leaderMotor.getVelocity().getValueAsDouble() * 60d;
    inputs.acceleration = leaderMotor.getAcceleration().getValueAsDouble() * 60d;

    inputs.leaderTemp = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.leaderVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.leaderStatorCurrent = leaderMotor.getStatorCurrent().getValueAsDouble();

    inputs.followerTemp = followerMotor.getDeviceTemp().getValueAsDouble();
    inputs.followerVoltage = followerMotor.getMotorVoltage().getValueAsDouble();
    inputs.followerStatorCurrent = followerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Velocity = (setpoint / 60d); // Convert RPM to rotations per second
  }

  @Override
  public boolean atSetpoint() {
    // Compare target (RPS) to actual velocity (converted back to RPM) within tolerance
    return MathUtil.isNear(
        setpoint.Velocity * 60d,
        leaderMotor.getVelocity().getValueAsDouble() * 60d,
        FlywheelConstants.VELOCITY_TOLERANCE);
  }
}
