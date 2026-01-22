// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tunnel;

<<<<<<< HEAD
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

=======
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
>>>>>>> 190c8c66425dae32f01cfe39dba5a07619e6c17e

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Tunnel", inputs);
  }
}
