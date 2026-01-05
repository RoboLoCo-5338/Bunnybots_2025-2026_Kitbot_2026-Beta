package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double shooterPosition = 0.0;
    public double shooterVelocity = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterCurrentAmps = 0.0;
    public double shooterCurrent = 0.0;
    public boolean shooterConnected = false;
    public double shooterTemperature = 0.0;
  }

  /**
   * Updates the set of loggable inputs for the arm subsystem. This function updates the following
   * inputs:
   *
   * <ul>
   *   <li>{@code armConnected}: Whether the arm motor is connected
   *   <li>{@code armPosition}: The position of the arm motor in radians
   *   <li>{@code armVelocity}: The velocity of the arm motor in radians per second
   *   <li>{@code armAppliedVolts}: The voltage applied to the arm motor in volts
   *   <li>{@code armCurrent}: The current drawn by the arm motor in amps
   * </ul>
   */
  /**
   * Updates the inputs for the shooter
   *
   * @param inputs the ShooterIO inputs
   */
  public void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Sets the shooter velocity
   *
   * @param velocity the velocity you want the shooter to be set to
   */
  public void setShooterVelocity(AngularVelocity velocity) {}

  /**
   * The voltage going into the motor
   *
   * @param voltage the amount of voltage
   */
  public void shooterOpenLoop(Voltage voltage) {}
}
