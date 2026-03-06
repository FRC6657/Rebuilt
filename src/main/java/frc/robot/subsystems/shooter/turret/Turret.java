// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem controlling the horizontal rotation of the shooter assembly. Uses MotionMagic or
 * position PID to rotate the turret to a target heading in degrees.
 */
public class Turret extends SubsystemBase {

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Turret(TurretIO io) {
    this.io = io;
  }

  /**
   * Creates a command that rotates the turret to a fixed angle.
   *
   * @param setpoint the target heading in degrees
   * @return the command
   */
  public Command changeSetpoint(double setpoint) {
    return this.runOnce(() -> io.changeSetpoint(setpoint));
  }

  public void changeSetpoint(double setpoint, double feedforward) {
    io.changeSetpoint(setpoint, feedforward);
  }

  /**
   * Creates a command that rotates the turret to a dynamically supplied angle.
   *
   * @param setpoint supplier providing the target heading in degrees
   * @return the command
   */
  public Command changeSetpoint(DoubleSupplier setpoint) {
    return this.runOnce(() -> io.changeSetpoint(setpoint.getAsDouble()));
  }

  public Command restingSetpoint() {
    return changeSetpoint(TurretConstants.restingSetpoint);
  }

  /**
   * @return true if the turret is within tolerance of its target angle
   */
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  /**
   * @return the current turret heading in degrees
   */
  public double getPosition() {
    return inputs.position;
  }

  /**
   * @return the current turret heading in degrees
   */
  public double getSetpoint() {
    return inputs.setpoint;
  }

  public void toggleControl() {
    io.toggleControl();
  }

  /** Updates sensor inputs and logs them to AdvantageKit each cycle. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);
  }
}
