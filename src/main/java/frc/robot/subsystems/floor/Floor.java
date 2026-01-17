package frc.robot.subsystems.floor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Floor extends SubsystemBase {

  private final FloorIO io;
  private final FloorIOInputsAutoLogged inputs = new FloorIOInputsAutoLogged();

  public Floor(FloorIO io) {
    this.io = io;
  }

  public Command changeRollerSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  public void setRollerSetpoint(double setpoint) {
    io.changeSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", (LoggableInputs) inputs);
  }
}
