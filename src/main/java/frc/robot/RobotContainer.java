// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonFX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);

  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOTalonFX(), new GyroIO());
        shooter = new Shooter(new ShooterIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO());
        shooter = new Shooter(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO(), new GyroIO());
        shooter = new Shooter(new ShooterIO());
        break;
    }
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    shooter.addRoutinesToChooser(autoChooser);

    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // operator control for shooter
    operatorController
        .rightTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_OUTTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));
    operatorController
        .leftTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_OUTTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));

    operatorController
        .leftBumper()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_INTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));
    operatorController
        .rightBumper()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_INTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));

    // driver control for shooter
    driveController
        .rightTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_OUTTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));
    driveController
        .leftTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_OUTTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));
    driveController
        .leftBumper()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_INTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));
    driveController
        .rightBumper()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.SHOOTER_INTAKE_VELOCITY)))
        .onFalse(shooter.setShooterVelocity(() -> DegreesPerSecond.of(0)));

    // driver control, arcade drive
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -driveController.getLeftY(), () -> -driveController.getRightX()));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
