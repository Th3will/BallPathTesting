// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final WPI_TalonFX upperFalcon = new WPI_TalonFX(10);
  private final WPI_TalonFX lowerFalcon = new WPI_TalonFX(9);

  private WPI_TalonFX frontMotor;
  private WPI_TalonFX rearMotor;

  private SimpleMotorFeedforward frontFeedforward;
  private SimpleMotorFeedforward rearFeedforward;

  private double prevTime;
  private double frontPrevOmega = 0.0;
  private double rearPrevOmega = 0.0;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putNumber("Feed Power", 0.0);

    upperFalcon.follow(lowerFalcon);
    lowerFalcon.setInverted(true);

    frontMotor = new WPI_TalonFX(0);
    rearMotor = new WPI_TalonFX(1);

    // Voltage Comp Satuation
    frontMotor.configVoltageCompSaturation(12);
    rearMotor.configVoltageCompSaturation(12);

    frontMotor.enableVoltageCompensation(false);
    rearMotor.enableVoltageCompensation(false);

    // Inversion
    frontMotor.setInverted(true);

    // PIDF
    frontMotor.config_kP(0, 0.01);
    rearMotor.config_kP(0, 0.01);

    frontFeedforward = new SimpleMotorFeedforward(0.67312, 0.017196, 0.0017592);
    rearFeedforward = new SimpleMotorFeedforward(0.67312, 0.017196, 0.0017592);

    // SmartDashboard
    SmartDashboard.putNumber("Shooter Power", 0.0);
    SmartDashboard.putNumber("Velocity Setpoint", 0.0);
    SmartDashboard.putBoolean("Power Mode", false);

    SmartDashboard.putNumber("Front Motor Velocity RPM", 0.0);
    SmartDashboard.putNumber("Rear Motor Velocity RPM", 0.0);

    SmartDashboard.putNumber("Front Motor Acceleration", 0.0);
    SmartDashboard.putNumber("Front Motor Velocity RadS", 0.0);

    prevTime = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("Shooter Power", 0.0);
    SmartDashboard.putNumber("Velocity Setpoint", 0.0);
    SmartDashboard.putBoolean("Power Mode", false);

    SmartDashboard.putNumber("Front Motor Velocity RPM", 0.0);
    SmartDashboard.putNumber("Rear Motor Velocity RPM", 0.0);

    SmartDashboard.putNumber("Front Motor Acceleration", 0.0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double curTime = Timer.getFPGATimestamp();

    lowerFalcon.set(SmartDashboard.getNumber("Feed Power", 0.0));
    SmartDashboard.putNumber("Top Current", upperFalcon.getStatorCurrent());
    SmartDashboard.putNumber("Bottom Current", lowerFalcon.getStatorCurrent());

    // Velocity
    SmartDashboard.putNumber("Front Motor Velocity RPM", getVelocityRPM(frontMotor));
    SmartDashboard.putNumber("Rear Motor Velocity RPM", getVelocityRPM(rearMotor));

    // MOI
    double frontOmega = getVelocityRadS(frontMotor);

    SmartDashboard.putNumber("Front Motor Acceleration", calculateWheelAcceleration(frontMotor, frontOmega, frontPrevOmega, curTime, prevTime));
    SmartDashboard.putNumber("Front Motor Velocity RadS", frontOmega);

    prevTime = curTime;
    frontPrevOmega = frontOmega;

    // Control Logic
    if (SmartDashboard.getBoolean("Power Mode", false)) {
      double power = SmartDashboard.getNumber("Shooter Power", 0.0);

      frontMotor.set(ControlMode.PercentOutput, power);
      rearMotor.set(ControlMode.PercentOutput, power);
    } else {
      double velocitySetpoint = SmartDashboard.getNumber("Velocity Setpoint", 0.0);

      frontMotor.set(ControlMode.Velocity, Conversions.RPMToFalcon(velocitySetpoint, 1), DemandType.ArbitraryFeedForward, frontFeedforward.calculate(Units.rotationsPerMinuteToRadiansPerSecond(velocitySetpoint), 0.0) / 12.0);
      rearMotor.set(ControlMode.Velocity, Conversions.RPMToFalcon(velocitySetpoint, 1), DemandType.ArbitraryFeedForward, rearFeedforward.calculate(Units.rotationsPerMinuteToRadiansPerSecond(velocitySetpoint), 0.0) / 12.0);
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static double calculateWheelAcceleration(WPI_TalonFX motor, double omega, double prevOmega, double curTime, double prevTime) {
    // Estimate the motor torque using Kt.
    double torque = motor.getStatorCurrent() * DCMotor.getFalcon500(1).KtNMPerAmp;

    // Estimate the acceleration using numerical differentaion.
    double acceleration = (omega - prevOmega) / (curTime - prevTime);

    // tau = I * alpha
    // Returns MOI in kg * m^2.
    return acceleration;
  }

  public static double getVelocityRPM(WPI_TalonFX motor) {
    return Conversions.falconToRPM(motor.getSelectedSensorVelocity(), 1);
  }

  public static double getVelocityRadS(WPI_TalonFX motor) {
    return Units.rotationsPerMinuteToRadiansPerSecond(getVelocityRPM(motor));
  }
}
