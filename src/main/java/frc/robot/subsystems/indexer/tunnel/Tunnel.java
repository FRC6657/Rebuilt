// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer.tunnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Tunnel indexer subsystem - a belt/roller mechanism that feeds game pieces vertically from the
 * floor indexer up to the shooter. Runs via voltage control.
 */
public class Tunnel extends SubsystemBase {

  private final TunnelIO io;
  private final TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Tunnel(TunnelIO io) {
    this.io = io;
  }

  /**
   * Creates a command that sets the tunnel roller voltage.
   *
   * @param setpoint the desired voltage setpoint (off, forward, reverse)
   * @return the command
   */
  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  /** Updates sensor inputs and logs them to AdvantageKit each cycle. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Tunnel", inputs);
  }
}
