// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TurretIO io;

  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
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

  @AutoLogOutput(key = "RobotStates/TurretAtSetpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.Setpoint, inputs.Position, 2);
  }

  public double getPosition() {
    return inputs.Position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("ArmPivot", inputs);
  }
}
