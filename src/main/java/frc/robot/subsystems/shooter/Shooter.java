package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIdSubsystem;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Shooter extends SubsystemBase implements SysIdSubsystem.SysIdSingleSubsystem {
  public final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();

  private final Alert shooterDisconnectedAlert =
      new Alert("Shooter motor disconnected!", AlertType.kError);

  private final SysIdRoutine sysIdRoutine;

  /**
   * A method that configures the base unit for velocity and voltage with a mechanism of the SysId
   * that finds the feed forward values for velocity and voltage
   *
   * @param io The shooters io
   */
  public Shooter(ShooterIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofBaseUnits(0.2, Volts.per(Second)),
                Voltage.ofBaseUnits(2, Volts),
                Second.of(10),
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new Mechanism(io::shooterOpenLoop, null, this));
  }

  /**
   * The method periodic updates and processes the input and gives an alert if the shooter is not
   * connected and it isn't in SIM mode
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", (LoggableInputs) inputs);

    shooterDisconnectedAlert.set(!inputs.shooterConnected && Constants.CURRENT_MODE != Mode.SIM);
  }

  /**
   * This method sets the shooter velocity with the name Set Shooter Velocity
   *
   * @param velocity The velocity that is set for the shooter
   * @return An instant command that set the shooter velocity
   */
  public Command setShooterVelocity(Supplier<AngularVelocity> velocity) {
    return new InstantCommand(
            () -> {
              io.setShooterVelocity(velocity.get());
            },
            this)
        .withName("Set Shooter Velocity");
  }

  /**
   * A method that gets the SysId
   *
   * @return the sysId
   */
  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  /**
   * A method that gets the name of the shooter
   *
   * @return Returns Shooter
   */
  @Override
  public String getName() {
    return "Shooter ";
  }

  /**
   * Resets the shooter velocity to 0 then idles till the velocity is less than 0.05
   *
   * @return Returns the shooter velocity to 0
   */
  @Override
  public Command reset(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> io.shooterOpenLoop(Volts.of(0)), this),
        Commands.idle(this).until(() -> Math.abs(inputs.shooterVelocity) < 0.05));
  }
}
