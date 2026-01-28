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
    Floor_One(16),
    Floor_Two(17),
    Hood(18),
    Turret(19),
    Intake_Extension(20),
    Intake_Wheels(21),
    Intake_Encoder(22);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }
}
