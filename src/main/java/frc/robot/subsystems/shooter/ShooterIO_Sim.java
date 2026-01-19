package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIO_Sim implements ShooterIO {
    private double voltage = 0;
    double setpoint = 0.0;
    double speed = 0.0;
    
    private DCMotorSim shooterSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(2), 0.0001, ShooterConstants.gearRatio),
          DCMotor.getFalcon500(2));

  private DCMotorSim shooterSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(2), 0.0001));

  public ShooterIO_Sim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    shooterSim.update(ShooterConstants.updateFrequency);

    ShooterIOInputs.position = shooterSim.getAngularPositionRad();
    ShooterIOInputs.velocity = shooterSim.getAngularVelocityRPM();
    ShooterIOInputs.leaderMotorVoltage = voltage;
    ShooterIOInputs.followerMotorVoltage = voltage;
    ShooterIOInputs.leaderMotorCurrent = shooterSim.getCurrentDrawAmps();
    ShooterIOInputs.setpoint = setpoint;
  }

  @Override
  public void changeSetpoint(double volts) {
    ShooterIOInputs.setpoint = setpoint;
  }
}
