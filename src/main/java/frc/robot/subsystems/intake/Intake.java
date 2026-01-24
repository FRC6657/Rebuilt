// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public IntakeIO io;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  /** Creates a new Intake. */
  public Intake() {
    this.io = io;
  }

  public Command changeExtSetpoint(double ext) {
    return this.runOnce(
        () ->
            io.changeExtSetpoint(
                MathUtil.clamp(
                    ext,
                    IntakeConstants.ExtensionMotor.minLength,
                    IntakeConstants.ExtensionMotor.maxLength)));
  }

  @AutoLogOutput(key = "Intake/AtExtensionSetpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(
        inputs.extMotorSetpoint, inputs.encoderAbsPosition, Units.inchesToMeters(0.5));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command changeWheelSpeed(double speed) {
    return this.runOnce(() -> io.changeWheelSetpoint(speed));
  }
}
