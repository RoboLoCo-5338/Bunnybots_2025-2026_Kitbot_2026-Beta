package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOTalonFX extends ShooterIO {
  private final StatusSignal<AngularVelocity> shooterVelocity;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Current> shooterCurrent;
  private final StatusSignal<Temperature> shooterTemperature;
  private final StatusSignal<Integer> shooterVersion;
  private final StatusSignal<Angle> shooterPosition;

  private final Debouncer shooterDebouncer = new Debouncer(0.5);

  public final TalonFX shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_1_ID);
  final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0.0);
  final VoltageOut shooterOpenLoop = new VoltageOut(0.0);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter kP", ShooterConstants.SHOOTER_MOTOR_VELOCITY_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter kI", ShooterConstants.SHOOTER_MOTOR_VELOCITY_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter kD", ShooterConstants.SHOOTER_MOTOR_VELOCITY_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter kV", ShooterConstants.SHOOTER_MOTOR_KV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter kS", ShooterConstants.SHOOTER_MOTOR_KS);

  public ShooterIOTalonFX() {
    shooterVelocity = shooterMotor.getVelocity();
    shooterAppliedVolts = shooterMotor.getMotorVoltage();
    shooterCurrent = shooterMotor.getStatorCurrent();
    shooterTemperature = shooterMotor.getDeviceTemp();
    shooterVersion = shooterMotor.getVersion();
    shooterPosition = shooterMotor.getPosition();

    shooterMotor.getConfigurator().apply(getShooterConfiguration());

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, shooterVelocity, shooterAppliedVolts, shooterCurrent, shooterTemperature));

    ParentDevice.optimizeBusUtilizationForAll(shooterMotor);
  }

  public TalonFXConfiguration getShooterConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kS = kS.get();

    config.Feedback.SensorToMechanismRatio = ShooterConstants.GEARING;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    config.CurrentLimits = currentConfig;
    return config;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            shooterVelocity,
            shooterCurrent,
            shooterAppliedVolts,
            shooterPosition,
            shooterTemperature);

    inputs.shooterConnected = shooterDebouncer.calculate(motor1Status.isOK());
    inputs.shooterVelocity = shooterVelocity.getValueAsDouble();
    inputs.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterCurrentAmps = shooterCurrent.getValueAsDouble();
    inputs.shooterTemperature = shooterTemperature.getValueAsDouble();
    inputs.shooterPosition = shooterPosition.getValueAsDouble();

    LoggedTunableNumber.ifChanged(
        0,
        () -> shooterMotor.getConfigurator().apply(getShooterConfiguration()),
        kP,
        kI,
        kD,
        kV,
        kS);
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    shooterMotor.setControl(shooterVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void shooterOpenLoop(Voltage voltage) {
    shooterMotor.setControl(shooterOpenLoop.withOutput(voltage));
  }
}
