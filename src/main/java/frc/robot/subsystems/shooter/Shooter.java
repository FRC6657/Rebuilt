package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public Command changeSetpoint(double Setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(Setpoint);
        });
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.setpoint, inputs.position, Units.degreesToRadians(5));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/", inputs);
  }
}
