package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

public class IntakeIO_Sim implements IntakeIO {
  private double extSetpoint = IntakeConstants.ExtensionMotor.minLength;
  private double speedSetpoint = 0;

  private ProfiledPIDController extPIDSim =
      new ProfiledPIDController(
          13.5, 0, 0, new Constraints(Units.inchesToMeters(12), Units.inchesToMeters(10)));

  private DCMotorSim extSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, 1), // TODO configure correct gearing
          DCMotor.getFalcon500(1));

  private DCMotorSim wheelSim = 
      new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, 1),
      DCMotor.getFalcon500(1));

  public IntakeIO_Sim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    extSim.update(1 / GlobalConstants.mainLoopFrequency);

    inputs.extMotorVelocity = extSim.getAngularVelocityRPM();
    inputs.extMotorVoltage = extSim.getInputVoltage();
    inputs.extMotorCurrent = extSim.getCurrentDrawAmps();
    inputs.extMotorSetpoint = extSetpoint;

    inputs.wheelMotorVelocity = wheelSim.getAngularVelocityRPM();
    inputs.wheelMotorVoltage = wheelSim.getInputVoltage();
    inputs.wheelMotorCurrent = wheelSim.getCurrentDrawAmps();
    inputs.wheelMotorSetpoint = speedSetpoint;
  }

  @Override
  public void changeExtSetpoint(double setpoint) {
    extSetpoint = setpoint;
  }

  @Override
  public void changeWheelSetpoint(double setpoint) {
    speedSetpoint = setpoint;
  }
}
