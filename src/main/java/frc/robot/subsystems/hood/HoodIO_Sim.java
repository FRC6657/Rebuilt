// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIO_Sim implements HoodIO {
  /** Creates a new HoodIO_Sim. */
  private double voltage = 0;

  private double setpoint = HoodConstants.INITIAL_SETPOINT;

  private DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, HoodConstants.GEAR_RATIO),
          DCMotor.getFalcon500(1));

  private PIDController hoodPID = new PIDController(1, 0, 0);

  public HoodIO_Sim() {
    hoodSim.setAngle(Units.degreesToRadians(-90));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // This method will be called once per scheduler run
    hoodSim.setInputVoltage(
        hoodPID.calculate(hoodSim.getAngularPositionRotations() * 360, setpoint));
    // Update Value With CodeConstants
    hoodSim.update(1);

    inputs.Velocity = hoodSim.getAngularVelocityRPM();
    inputs.Position = hoodSim.getAngularPositionRotations() * 360;
    inputs.Temp = 0;
    inputs.Voltage = voltage;
    inputs.Current = hoodSim.getCurrentDrawAmps();
    inputs.Setpoint = setpoint;
  }
}
