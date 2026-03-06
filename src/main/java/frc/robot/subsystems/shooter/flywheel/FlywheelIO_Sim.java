package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

/**
 * Simulated flywheel implementation using DCMotorSim physics. The leader uses TalonFX sim state for
 * velocity PID; the follower mirrors via Follower control.
 */
public class FlywheelIO_Sim implements FlywheelIO {
  private TalonFX leader = new TalonFX(GlobalConstants.CAN.Shooter_Leader.id);
  private TalonFX follower = new TalonFX(GlobalConstants.CAN.Shooter_Follower.id);

  private VelocityVoltage setpoint = new VelocityVoltage(0);

  private DCMotorSim motorModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              FlywheelConstants.MOTOR, 0.001, FlywheelConstants.GEAR_RATIO),
          FlywheelConstants.MOTOR);

  public FlywheelIO_Sim() {

    var simCurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimitEnable(false);
    leader.getConfigurator().apply(FlywheelConstants.CONFIG.withCurrentLimits(simCurrentLimits));
    follower.getConfigurator().apply(FlywheelConstants.CONFIG.withCurrentLimits(simCurrentLimits));

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

    var motorSim = leader.getSimState();
    motorSim.setSupplyVoltage(12);
    motorModel.setInputVoltage(motorSim.getMotorVoltage());
    motorModel.update(1 / GlobalConstants.mainLoopFrequency);
    var angularPosition = motorModel.getAngularPosition().times(FlywheelConstants.GEAR_RATIO);
    var angularVelocity = motorModel.getAngularVelocity().times(FlywheelConstants.GEAR_RATIO);
    motorSim.setRawRotorPosition(angularPosition);
    motorSim.setRotorVelocity(angularVelocity);

    var followerSim = follower.getSimState();
    followerSim.setSupplyVoltage(12);
    followerSim.setRawRotorPosition(angularPosition);
    followerSim.setRotorVelocity(angularVelocity);

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
    this.setpoint.Velocity = (setpoint / 60d); // RPM to RPS
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(
        setpoint.Velocity * 60d,
        leader.getVelocity().getValueAsDouble() * 60d,
        FlywheelConstants.VELOCITY_TOLERANCE);
  }
}
