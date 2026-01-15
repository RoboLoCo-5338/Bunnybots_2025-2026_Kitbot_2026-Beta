package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {
  public static final int SHOOTER_MOTOR_1_ID = 4;
  public static final double SHOOTER_MOTOR_VELOCITY_KP = 0.2;
  public static final double SHOOTER_MOTOR_VELOCITY_KI = 0.01;
  public static final double SHOOTER_MOTOR_VELOCITY_KD = 0;
  public static final double SHOOTER_MOTOR_KV = 0.42478;
  public static final double SHOOTER_MOTOR_KS = 0.098161;
  public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 60;
  public static final double GEARING = 4.0;

  public static final double SHOOTER_INTAKE_VELOCITY = 5.0;
  public static final double SHOOTER_OUTTAKE_VELOCITY = 15.0;

  public static final class ShooterSimConstants {
    public static final MomentOfInertia SHOOTER_MOI =
        Pounds.mult(InchesPerSecond).mult(Inches).per(RadiansPerSecond).of(1.517);
  }
}
