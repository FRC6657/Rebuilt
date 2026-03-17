package frc.robot;

public class GlobalConstants {

  public static double mainLoopFrequency = 50d;

  public static enum CAN {
    Swerve_FL_D(1),
    Swerve_FR_D(2),
    Swerve_BL_D(3),
    Swerve_BR_D(4),
    Swerve_FL_T(5),
    Swerve_FR_T(6),
    Swerve_BL_T(7),
    Swerve_BR_T(8),
    Swerve_FL_E(9),
    Swerve_FR_E(10),
    Swerve_BL_E(11),
    Swerve_BR_E(12),
    Swerve_Gyro(13),
    Shooter_Leader(14),
    Shooter_Follower(15),
    Floor_One(16),
    Hood(17),
    Turret(18),
    Intake_Extension(19),
    Intake_Wheels(20),
    Tunnel(21),
    Climber(22),
    Floor_Two(23);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static enum opButtons {
    KnobPush(1),
    Circle(2),
    Triangle(3),
    Square(4),
    Cross(5),
    M1(6),
    NumClr(7),
    Slash(8),
    Star(9),
    Minus(10),
    M2(11),
    Seven(12),
    Eight(13),
    Nine(14),
    Plus(15),
    M3(16),
    Four(17),
    Five(18),
    Six(19),
    M4(20),
    One(21),
    Two(22),
    Three(23),
    M5(24),
    Zero(25),
    Period(26),
    Enter(27),
    KnobLeft(28),
    KnobRight(29);

    public int id;

    opButtons(int button) {
      this.id = button;
    }
  }
}
