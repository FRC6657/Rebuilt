package frc.robot.subsystems.tunnel;

public interface TunnelIO {

    public static class TunnelIOInputs{
    public static double velocity = 0.0;
    public static double temp = 0.0;
    public static double voltage = 0.0;
    public static double current = 0.0;
    public static double setpoint = 0.0;
    }

  public void updateInputs(TunnelIOInputs inputs){}
  public void changeRollerSpeed(double speed){}
}
