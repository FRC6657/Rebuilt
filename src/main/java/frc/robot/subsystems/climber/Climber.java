// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public Command changeSetpoint(double newSetpoint) {
    return runOnce(() -> io.changeSetpoint(newSetpoint));
  }

  @AutoLogOutput(key = "MechanismStates/ClimberAtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.positionSetpoint - inputs.motorPosition)
      < ClimberConstants.HEIGHT_TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber/", inputs);
  }
}
