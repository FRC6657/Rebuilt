package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIO_Sim implements IntakeIO {

    private ProfiledPIDController extPID = 
        new ProfiledPIDController(13.5, 0, 0, new Constraints(Units.inchesToMeters(12), Units.inchesToMeters(10))
    );

    private DCMotorSim extSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, 1), //TODO configure correct gearing
        DCMotor.getFalcon500(1)
    );

    public IntakeIO_Sim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    @Override
    public void changeExtSetpoint(double setpoint) {

    }
}