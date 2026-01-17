package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double extMotorPosition = 0.0; // Inches
    public double extMotorVelocity = 0.0; // Inches per second
    public double extMotorTemp = 0.0; // Celcius
    public double extMotorVoltage = 0.0; // Volts
    public double extMotorCurrent = 0.0; // Amps
    public double extMotorSetpoint = 0.0; // 0 to 6

    public double encoderAbsPosition = 0.0;
    public double encoderRelPosition = 0.0;
    public double encoderVelocity = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void changeExtSetpoint(double setpoint) {}
}
