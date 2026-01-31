package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;
import frc.robot.subsystems.intake.IntakeConstants.Roller.RollerSetpoint;

/**
 * Simulated implementation of IntakeIO using DCMotorSim models. Mirrors the real implementation's
 * PID logic while simulating motor physics.
 */
public class IntakeIO_Sim implements IntakeIO {

  private TalonFX extensionMotor = new TalonFX(GlobalConstants.CAN.Intake_Extension.id);
  private TalonFX rollerMotor = new TalonFX(GlobalConstants.CAN.Intake_Wheels.id);

  private VoltageOut rollerSetpoint = new VoltageOut(0);
  private ExtensionSetpoint extensionSetpoint = ExtensionSetpoint.RETRACTED_FAST;

  // WPILib profiled PID for smooth trapezoidal extension motion
  private ProfiledPIDController extensionPID =
      new ProfiledPIDController(
          13.5, 0, 0, new Constraints(extensionSetpoint.velocity, extensionSetpoint.acceleration));

  // Physics simulation for the extension motor
  private DCMotorSim extensionSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, IntakeConstants.Extension.GEAR_RATIO),
          DCMotor.getFalcon500(1));

  // Physics simulation for the roller motor
  private DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, IntakeConstants.Roller.GEAR_RATIO),
          DCMotor.getFalcon500(1));

  public IntakeIO_Sim() {
    extensionMotor.getConfigurator().apply(IntakeConstants.Extension.CONFIG);
    rollerMotor.getConfigurator().apply(IntakeConstants.Roller.CONFIG);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Run extension PID controller and apply voltage output
    extensionMotor.setControl(
        new VoltageOut(
            extensionPID.calculate(
                extensionMotor.getPosition().getValueAsDouble()
                    * IntakeConstants.Extension.CONVERSION_FACTOR)));

    // Step the extension physics simulation forward
    var extensionMotorSim = extensionMotor.getSimState();
    extensionMotorSim.setSupplyVoltage(12);
    extensionSim.setInputVoltage(extensionMotorSim.getMotorVoltage());
    extensionSim.update(1 / GlobalConstants.mainLoopFrequency);
    // Feed simulated position/velocity back into the TalonFX sim state (rotor units)
    extensionMotorSim.setRawRotorPosition(
        extensionSim
            .getAngularPosition()
            .times(
                IntakeConstants.Extension.GEAR_RATIO
                    / IntakeConstants.Extension.CONVERSION_FACTOR));
    extensionMotorSim.setRotorVelocity(
        extensionSim
            .getAngularVelocity()
            .times(
                IntakeConstants.Extension.GEAR_RATIO
                    / IntakeConstants.Extension.CONVERSION_FACTOR));

    // Apply roller voltage
    rollerMotor.setControl(rollerSetpoint);

    // Step the roller physics simulation forward
    var rollerMotorSim = rollerMotor.getSimState();
    rollerMotorSim.setSupplyVoltage(12);
    rollerSim.setInputVoltage(rollerMotorSim.getMotorVoltage());
    rollerSim.update(1 / GlobalConstants.mainLoopFrequency);
    rollerMotorSim.setRawRotorPosition(
        rollerSim.getAngularPosition().times(IntakeConstants.Roller.GEAR_RATIO));
    rollerMotorSim.setRotorVelocity(
        rollerSim.getAngularVelocity().times(IntakeConstants.Roller.GEAR_RATIO));

    inputs.extensionPosition = rollerMotor.getPosition().getValueAsDouble();
    inputs.extensionVelocity = rollerMotor.getVelocity().getValueAsDouble();
    inputs.extensionAcceleration = rollerMotor.getAcceleration().getValueAsDouble();
    inputs.extensionVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(ExtensionSetpoint setpoint) {
    extensionPID.reset(
        extensionMotor.getPosition().getValueAsDouble()
            * IntakeConstants.Extension.CONVERSION_FACTOR,
        extensionMotor.getVelocity().getValueAsDouble()
            * IntakeConstants.Extension.CONVERSION_FACTOR);
    extensionPID.setConstraints(new Constraints(setpoint.velocity, setpoint.acceleration));
    extensionPID.setGoal(setpoint.position);
  }

  @Override
  public void changeSetpoint(RollerSetpoint setpoint) {
    rollerSetpoint.Output = setpoint.voltage;
  }
}
