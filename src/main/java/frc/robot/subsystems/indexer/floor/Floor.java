package frc.robot.subsystems.indexer.floor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.floor.FloorConstants.FloorSetpoint;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Floor indexer subsystem - a belt/roller mechanism at the bottom of the indexer that moves game
 * pieces from the intake toward the tunnel. Runs via voltage control.
 */
public class Floor extends SubsystemBase {

  private final FloorIO io;
  private final FloorIOInputsAutoLogged inputs = new FloorIOInputsAutoLogged();

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Floor(FloorIO io) {
    this.io = io;
  }

  /**
   * Creates a command that sets the floor roller voltage.
   *
   * @param setpoint the desired voltage setpoint (off, forward, reverse)
   * @return the command
   */
  public Command changeSetpoint(FloorSetpoint setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  /** Updates sensor inputs and logs them to AdvantageKit each cycle. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Floor", (LoggableInputs) inputs);
  }
}
