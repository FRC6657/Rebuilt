package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIO_Real implements ShooterIO {
  TalonFX leaderMotor = new TalonFX(ShooterConstants.leaderCAN);
  TalonFX followerMotor = new TalonFX(ShooterConstants.followerCAN);

  private double ShooterSetpoint;

  public ShooterIO_Real() {

    var leaderConfigurator = leaderMotor.getConfigurator();
    var followerConfigurator = followerMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();

    motorConfigs.CurrentLimits = ShooterConstants.currentConfigs;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // can be changed later
    leaderConfigurator.apply(motorConfigs);
    followerConfigurator.apply(motorConfigs);
    followerMotor.setControl(new Follower(ShooterConstants.followerCAN, null));

    var leaderMotorPostition = leaderMotor.getPosition();
    var leaderMotorVelocity = leaderMotor.getVelocity();
    var leaderMotorAcceleration = leaderMotor.getAcceleration();
    var leaderMotorTemp = leaderMotor.getDeviceTemp();
    var leaderMotorVoltage = leaderMotor.getMotorVoltage();
    var leaderMotorCurrent = leaderMotor.getSupplyCurrent();

    var followerMotorTemp = followerMotor.getDeviceTemp();
    var followerMotorVoltage = followerMotor.getMotorVoltage();
    var followerMotorCurrent = followerMotor.getSupplyCurrent();

    leaderMotorTemp.setUpdateFrequency(ShooterConstants.updateFrequency);
    followerMotorTemp.setUpdateFrequency(ShooterConstants.updateFrequency);
    leaderMotorPostition.setUpdateFrequency(ShooterConstants.updateFrequency);
    leaderMotorVelocity.setUpdateFrequency(ShooterConstants.updateFrequency);
    leaderMotorAcceleration.setUpdateFrequency(ShooterConstants.updateFrequency);
    leaderMotorVoltage.setUpdateFrequency(ShooterConstants.updateFrequency);
    followerMotorVoltage.setUpdateFrequency(ShooterConstants.updateFrequency);
    leaderMotorCurrent.setUpdateFrequency(ShooterConstants.updateFrequency);
    followerMotorCurrent.setUpdateFrequency(ShooterConstants.updateFrequency);

    leaderMotor.setPosition(0);

    changeSetpoint(ShooterIOInputs.setpoint);
  }

  public void updateInputs(ShooterIO inputs) {
    ShooterIOInputs.setpoint = ShooterSetpoint;
    ShooterIOInputs.velocity = leaderMotor.getVelocity().getValueAsDouble();
    ShooterIOInputs.acceleration = leaderMotor.getAcceleration().getValueAsDouble();
  }

  public void changeSetpoint(double Setpoint) {}
}

