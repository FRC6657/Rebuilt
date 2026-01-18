// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  public Command changeSetpoint(DoubleSupplier setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint.getAsDouble());
        });
  }

  public void setpoint(double setpoint) {
    io.changeSetpoint(setpoint);
  }

  @AutoLogOutput(key = "RobotStates/HoodAtSetpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.Setpoint, inputs.Position, 2);
  }

  public double getPosition() {
    return inputs.Position;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }
}
