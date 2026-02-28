// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem controlling a linear extension mechanism and roller wheels. Uses the IO layer
 * pattern for hardware abstraction (real vs. simulated).
 */
public class Intake extends SubsystemBase {

  public IntakeIO io;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  /**
   * Creates a command that changes the intake extension setpoint (position, velocity,
   * acceleration).
   *
   * @param setpoint the desired extension setpoint
   * @return the command
   */
  public Command changeSetpoint(ExtensionSetpoint setpoint) {
    return this.runOnce(() -> io.changeSetpoint(setpoint));
  }

  /**
   * Creates a command that changes the intake roller voltage setpoint.
   *
   * @param setpoint the desired roller setpoint (off, forward, reverse)
   * @return the command
   */
  public Command changeSetpoint(double setpoint) {
    return this.runOnce(() -> io.changeSetpoint(setpoint));
  }

  /**
   * @return the current extension position in inches
   */
  public double getPosition() {
    return inputs.extensionPosition;
  }

  /**
   * @return true if the extension is within tolerance of its target position
   */
  @AutoLogOutput(key = "AtSetpoint/Intake")
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  /** Updates sensor inputs and logs them to AdvantageKit each cycle. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
