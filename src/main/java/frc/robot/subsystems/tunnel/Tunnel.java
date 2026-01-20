// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tunnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tunnel.TunnelIO;
import frc.robot.subsystems.tunnel.TunnelIO.TunnelIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;

public class Tunnel extends SubsystemBase {

  private final TunnelIO io;
  private final TunnelIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  
  public Tunnel(TunnelIO io) {
    this.io = io;
  }

  public Command changeRollerSpeed(double speed) {
    
  }

  @AutoLogOutput(key = "Tunnel/AtSetpoint")
  public boolean atSetpoint(){
    return MathUtil.isNear(inputs.)
  }

  @Override
  public void periodic() {
    
  }

}
