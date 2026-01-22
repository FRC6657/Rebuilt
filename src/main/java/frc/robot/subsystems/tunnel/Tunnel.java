// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tunnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.System.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.AutoLogOutput;

public class Tunnel extends SubsystemBase {

  private final TunnelIO io;
  private final TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();

  public Tunnel(TunnelIO io) {
    this.io = io;
  }

  public Command changeRollerSpeed(double speed) {
    return this.runOnce(
        () -> {
          io.setSpeed(speed);
        });
  }

  @AutoLogOutput(key = "Tunnel/AtSetpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.setpoint, inputs.getPosition);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Tunnel", inputs);
  }
}
