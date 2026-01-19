package frc.robot.subsystems.floor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FloorIO_Real implements FloorIO {

  TalonFX rollerMotor = new TalonFX(2);
  double rollerSetpoint = 0;

  public FloorIO_Real() {

    var motorConfigurator = rollerMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigurator.apply(motorConfigs);

    var Temp = rollerMotor.getDeviceTemp();
    var Voltage = rollerMotor.getMotorVoltage();
    var Current = rollerMotor.getSupplyCurrent();

    Temp.setUpdateFrequency(50 / 4); // TODO: use the constant
    Voltage.setUpdateFrequency(50);
    Current.setUpdateFrequency(50);

    rollerMotor.optimizeBusUtilization();

    changeSetpoint(0);
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {

    rollerMotor.setControl(new VoltageOut(rollerSetpoint));

    inputs.temp = rollerMotor.getDeviceTemp().getValueAsDouble();
    inputs.current = rollerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.voltage = rollerMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    rollerSetpoint = setpoint;
    rollerMotor.setControl(new VoltageOut(rollerSetpoint * 12));
  }
}
