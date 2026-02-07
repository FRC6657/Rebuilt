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

  private TalonFX leader = new TalonFX(GlobalConstants.CAN.Shooter_Leader.id);
  private TalonFX follower = new TalonFX(GlobalConstants.CAN.Shooter_Follower.id);

  private VelocityVoltage setpoint = new VelocityVoltage(0);

  public FlywheelIO_Real() {

    leader.getConfigurator().apply(FlywheelConstants.CONFIG);
    follower.getConfigurator().apply(FlywheelConstants.CONFIG);

    var velocity = leader.getVelocity();
    var acceleration = leader.getAcceleration();

    var leaderTemp = leader.getDeviceTemp();
    var leaderVoltage = leader.getMotorVoltage();
    var leaderStatorCurrent = leader.getSupplyCurrent();

    var followerTemp = follower.getDeviceTemp();
    var followerVoltage = follower.getMotorVoltage();
    var followerStatorCurrent = follower.getSupplyCurrent();

    velocity.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    acceleration.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    leaderTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    followerTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    leaderVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    followerVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    leaderStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    followerStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    if (setpoint.Velocity == 0) {
      leader.setControl(new VoltageOut(0));
    } else {
      leader.setControl(setpoint);
    }
    follower.setControl(
        new Follower(GlobalConstants.CAN.Shooter_Leader.id, MotorAlignmentValue.Opposed));

    inputs.velocity = leader.getVelocity().getValueAsDouble() * 60d;
    inputs.acceleration = leader.getAcceleration().getValueAsDouble() * 60d;

    inputs.leaderTemp = leader.getDeviceTemp().getValueAsDouble();
    inputs.leaderVoltage = leader.getMotorVoltage().getValueAsDouble();
    inputs.leaderStatorCurrent = leader.getStatorCurrent().getValueAsDouble();

    inputs.followerTemp = follower.getDeviceTemp().getValueAsDouble();
    inputs.followerVoltage = follower.getMotorVoltage().getValueAsDouble();
    inputs.followerStatorCurrent = follower.getStatorCurrent().getValueAsDouble();
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
        leader.getVelocity().getValueAsDouble() * 60d,
        FlywheelConstants.VELOCITY_TOLERANCE);
  }
}
