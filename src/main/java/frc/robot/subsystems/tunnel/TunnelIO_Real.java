package frc.robot.subsystems.tunnel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TunnelIO_Real implements TunnelIO {

    TalonFX tunnelMotor = new TalonFX(TunnelConstants.TUNNEL_MOTOR);

    private double tunnelSetpoint = TunnelConstants.INITIAL_SETPOINT;

    public TunnelIO_Real(){

        var motorConfigurator = tunnelMotor.getConfigurator();
        var motorConfigs = new TalonFXConfiguration();

        motorConfigs.CurrentLimits = TunnelConstants.CURRENT_CONFIGS;

        public void updateInputs(TunnelIOInputs inputs){
            TunnelIOInputs.setpoint = tunnelSetpoint;
            TunnelIOInputs.velocity = tunnelMotor.getVelocity().getValueAsDouble();
        }

        public void changeRollerSpeed(double speed) {}
    
    }

    }

