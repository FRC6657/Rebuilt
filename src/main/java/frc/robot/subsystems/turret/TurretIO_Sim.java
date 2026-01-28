// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIO_Sim implements TurretIO {
  /** Creates a new TurretIO_Sim. */
  private double voltage = 0;

  private double setpoint = TurretConstants.INITIAL_SETPOINT * 360;

  private DCMotorSim turretSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, TurretConstants.GEAR_RATIO),
          DCMotor.getFalcon500(1));

  private PIDController turretPID = new PIDController(1, 0, 0);

  public TurretIO_Sim() {
    turretSim.setAngle(Units.degreesToRadians(-90));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // This method will be called once per scheduler run
    turretSim.setInputVoltage(
        turretPID.calculate(turretSim.getAngularPositionRotations() * 360, setpoint));
    // Update Value With CodeConstants
    turretSim.update(1);

    inputs.Velocity = turretSim.getAngularVelocityRPM();
    inputs.Position = turretSim.getAngularPositionRotations() * 360;
    inputs.Temp = 0;
    inputs.Voltage = voltage;
    inputs.Current = turretSim.getCurrentDrawAmps();
    inputs.Setpoint = setpoint;
  }

  @Override
  public void changeSetpoint(double setpoint) {
    double clampedInput = setpoint % 360;
    if (clampedInput < 0) {
      clampedInput = clampedInput + 360;
    }
    setpoint = clampedInput / 360 + TurretConstants.INITIAL_SETPOINT;
  }
}
