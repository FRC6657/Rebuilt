// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem controlling the shooter's launch angle via position PID. Adjusts the angle of the
 * shooter mechanism to control shot trajectory.
 */
public class Hood extends SubsystemBase {

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Hood(HoodIO io) {
    this.io = io;
  }

  /**
   * Creates a command that sets the hood to a target angle.
   *
   * @param setpoint the desired hood angle in degrees
   * @return the command
   */
  public Command changeSetpointC(double setpoint) {
    return this.runOnce(() -> io.changeSetpoint(setpoint));
  }

  public void changeSetpoint(double setpoint) {
    io.changeSetpoint(setpoint);
  }

  /**
   * @return true if the hood is within tolerance of its target angle
   */
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  /**
   * @return the current hood angle in degrees
   */
  public double getPosition() {
    return inputs.position;
  }

  /** Updates sensor inputs and logs them to AdvantageKit each cycle. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);
  }
}
