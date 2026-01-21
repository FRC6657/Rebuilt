import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class HoodIO_Sim implements HoodIO {
    public double voltage = 0;

    private double setpoint = HoodConstants.INITIAL_SETPOINT;

    private DCMotorSim hoodSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, HoodConstants.GEAR_RATIO))
}
