package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel shooter subsystem using a leader/follower motor pair. Controlled via velocity PID to
 * spin game pieces up to shooting speed.
 */
public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  /**
   * Creates a command that sets the flywheel target velocity.
   *
   * @param setpoint the desired flywheel voltage (V)
   * @return the command
   */
  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  /**
   * @return true if the flywheel velocity is within tolerance of the target
   */
  @AutoLogOutput(key = "AtSetpoint/Flywheel")
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  /** Updates sensor inputs and logs them to AdvantageKit each cycle. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);
  }
}
