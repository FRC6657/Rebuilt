package frc.robot;

public class GlobalConstants {

  public static double mainLoopFrequency = 50d;

  public static enum CAN {
    Swerve_FR_D(1),
    Swerve_FL_D(2),
    Swerve_BR_D(3),
    Swerve_BL_D(4),
    Swerve_FR_T(5),
    Swerve_FL_T(6),
    Swerve_BR_T(7),
    Swerve_BL_T(8),
    Swerve_FR_E(9),
    Swerve_FL_E(10),
    Swerve_BR_E(11),
    Swerve_BL_E(12),
    Swerve_Gyro(13),
    Shooter_Leader(14),
    Shooter_Follower(15),
    Floor(16),
    Hood(17),
    Turret(18),
    Intake_Extension(19),
    Intake_Wheels(20),
    Intake_Encoder(21),
    Tunnel(22);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }
}
