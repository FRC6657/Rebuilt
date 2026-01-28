// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.GlobalConstants;

public class TurretIO_Real implements TurretIO {
  TalonFX turretMotor = new TalonFX(GlobalConstants.CAN.Turret.id);
  private MotionMagicVoltage motionMagicVoltage =
      new MotionMagicVoltage(TurretConstants.INITIAL_SETPOINT);

  public TurretIO_Real() {

    var motorConfigurator = turretMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Slot0 = TurretConstants.motorSlot0;
    motorConfigs.CurrentLimits = TurretConstants.currentConfigs;
    motorConfigs.MotionMagic = TurretConstants.MotionMagicConfig;
    motorConfigurator.apply(motorConfigs);

    var Position = turretMotor.getPosition();
    var Temp = turretMotor.getDeviceTemp();
    var Voltage = turretMotor.getMotorVoltage();
    var Current = turretMotor.getSupplyCurrent();

    // update code with Global Constants
    Temp.setUpdateFrequency(50);
    Voltage.setUpdateFrequency(50);
    Current.setUpdateFrequency(50);
    Position.setUpdateFrequency(50);

    turretMotor.optimizeBusUtilization();

    //turretMotor.setPosition(TurretConstants.INITIAL_SETPOINT); // I don't think we need this

    changeSetpoint(TurretConstants.INITIAL_SETPOINT * 360);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    inputs.Temp = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.Current = turretMotor.getSupplyCurrent().getValueAsDouble();
    inputs.Voltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.Position = turretMotor.getPosition().getValueAsDouble() * 360;
    inputs.Velocity = turretMotor.getVelocity().getValueAsDouble() * 360;
    inputs.Accerleration = turretMotor.getAcceleration().getValueAsDouble() * 360;
    inputs.Setpoint = motionMagicVoltage.Position * 360;

    turretMotor.setControl(motionMagicVoltage);
  }

  @Override
  public void changeSetpoint(double setpoint){
    double clampedInput = setpoint % 360;
    if(clampedInput < 0){
      clampedInput = clampedInput + 360;
    }
    motionMagicVoltage = new MotionMagicVoltage(clampedInput/360 + TurretConstants.INITIAL_SETPOINT);
  }
}
