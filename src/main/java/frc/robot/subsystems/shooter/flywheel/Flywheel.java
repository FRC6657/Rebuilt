package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  public Command changeSetpointC(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  public void changeSetpoint(double setpoint) {
    io.changeSetpoint(setpoint);
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

  public Command sysIdRoutine() {
    var routine = makeSysIdRoutine();
    return Commands.sequence(
        routine.quasistatic(Direction.kForward).withTimeout(12),
        waitSeconds(5),
        routine.dynamic(Direction.kForward).withTimeout(5),
        waitSeconds(5),
        routine.quasistatic(Direction.kReverse).withTimeout(12),
        waitSeconds(5),
        routine.dynamic(Direction.kReverse).withTimeout(5),
        waitSeconds(5));
  }

  private SysIdRoutine makeSysIdRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            Seconds.of(12),
            state -> SignalLogger.writeString("ShooterSysIDState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> {
              changeSetpoint((volts.in(Volts)));
            },
            log -> {},
            this));
  }
}
