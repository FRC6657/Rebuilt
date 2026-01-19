package frc.robot.subsystems.tunnel;

public interface TunnelIO {

    public static class TunnelIOInputs{
    public double velocity = 0.0;
    public double temp = 0.0;
    public double voltage = 0.0;
    public double current = 0.0;
    public double setpoint = 0.0;
    }

  public void updateInputs(TunnelIOInputs inputs){}

  public void changeRollerSpeed(double speed) {}
}
