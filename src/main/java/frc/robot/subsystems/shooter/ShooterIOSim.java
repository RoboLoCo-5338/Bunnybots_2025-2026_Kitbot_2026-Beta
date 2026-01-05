package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim extends ShooterIOTalonFX {
  TalonFXSimState simMotor = shooterMotor.getSimState();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              ShooterSimConstants.SHOOTER_MOI.in(KilogramSquareMeters),
              ShooterConstants.GEARING),
          DCMotor.getKrakenX60(1));

  public ShooterIOSim() {
    super();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    Logger.recordOutput("Shooter/shooterVelocity", physicsSim.getAngularVelocityRPM());
    Logger.recordOutput("Shooter/shooterAppliedVolts", physicsSim.getInputVoltage());
    Logger.recordOutput("Shooter/shooterCurrentAmps", physicsSim.getCurrentDrawAmps());

    physicsSim.update(0.02);

    simMotor.addRotorPosition(physicsSim.getAngularVelocityRadPerSec() * 0.02);
    simMotor.setRotorVelocity(physicsSim.getAngularVelocityRadPerSec());

    super.updateInputs(inputs);
  }
}
