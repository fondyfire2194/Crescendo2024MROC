// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class ClimberSubsystem extends SubsystemBase implements Logged {
  /** Creates a new Climber. */
  CANSparkMax climberMotorLeft;
  CANSparkMax climberMotorRight;
  Servo climberLock;

  RelativeEncoder climberEncoderLeft;
  RelativeEncoder climberEncoderRight;

  public boolean leftMotorConnected;
  public boolean rightMotorConnected;
  @Log.NT(key = "simpositionleft")
  public double simpositionleft;
  @Log.NT(key = "simpositionright")
  private double simpositionright;
  @Log.NT(key = "currentspeedleft")
  private double currentSpeedLeft;
  @Log.NT(key = "currentspeedright")
  private double currentSpeedRight;
  @Log.NT(key = "erroright")
  private double errorright;

  public ClimberSubsystem() {
    climberMotorLeft = new CANSparkMax(CANIDConstants.climberIDLeft, MotorType.kBrushless);
    climberMotorRight = new CANSparkMax(CANIDConstants.climberIDRight, MotorType.kBrushless);
    climberEncoderLeft = climberMotorLeft.getEncoder();
    climberEncoderRight = climberMotorRight.getEncoder();

    climberLock = new Servo(0);
    unlockClimber();
    configMotor(climberMotorRight, climberEncoderRight, false);
    configMotor(climberMotorLeft, climberEncoderLeft, true);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kMinimal);
    motor.setSmartCurrentLimit(Constants.ClimberConstants.climberContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ClimberConstants.climberIdleMode);
    encoder.setVelocityConversionFactor(Constants.ClimberConstants.climberConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ClimberConstants.climberConversionPositionFactor);
    motor.enableVoltageCompensation(Constants.ClimberConstants.voltageComp);
    motor.setOpenLoopRampRate(1);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!leftMotorConnected) {
      leftMotorConnected = checkMotorCanOK(climberMotorLeft);
      SmartDashboard.putBoolean("Climber//OKLClimber", leftMotorConnected);
    }

    if (!rightMotorConnected) {
      rightMotorConnected = checkMotorCanOK(climberMotorRight);
      SmartDashboard.putBoolean("Climber//OKRClimber", rightMotorConnected);
    }
  }

  private boolean checkMotorCanOK(CANSparkMax motor) {
    double temp = motor.getOpenLoopRampRate();
    return RobotBase.isSimulation() || motor.setOpenLoopRampRate(temp) == REVLibError.kOk;
  }

  public Command testCan() {
    return Commands.parallel(
        Commands.runOnce(() -> leftMotorConnected = false),
        runOnce(() -> rightMotorConnected = false));
  }

  public void stopMotors() {
    stopLeftMotor();
    stopRightMotor();
  }

  public void stopRightMotor() {
    climberMotorRight.stopMotor();
    climberMotorRight.setVoltage(0);
    currentSpeedRight = 0;
  }

  public void stopLeftMotor() {
    climberMotorLeft.stopMotor();
    climberMotorLeft.setVoltage(0);
    currentSpeedLeft = 0;

  }

  public Command stopClimberCommand() {
    return Commands.parallel(
        stopLeftClimberCommand(),
        stopRightClimberCommand());
  }

  public Command stopLeftClimberCommand() {
    return Commands.runOnce(() -> stopLeftMotor());
  }

  public Command stopRightClimberCommand() {
    return Commands.runOnce(() -> stopRightMotor());
  }

  public void runLeftClimberMotor(double speed) {
    if (getPositionLeft() > 13.0) {
      speed = speed * 0.5;
    }
    currentSpeedLeft = speed;
    climberMotorLeft.setVoltage(speed * RobotController.getBatteryVoltage());

  }

  public void runRightClimberMotor(double speed) {
    if (getPositionRight() > 13.0) {
      speed = speed * 0.5;
    }
    currentSpeedRight = speed;
    climberMotorRight.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void runClimberMotors(double speed) {
    runRightClimberMotor(speed);
    runLeftClimberMotor(speed);
  }

  public void raiseClimber(double speed) {
    runClimberMotors(speed * 0.2);
    if (getPositionLeft() > 100.0) {
      runClimberMotors(speed * 0.2);
    } else {
      runClimberMotors(speed);
    }
  }

  public void lowerClimber(double speed) {
    speed *= -1;
    runClimberMotors(speed * 0.2);
    if (getPositionLeft() < 10) {
      runClimberMotors(speed * 0.2);
    } else {
      runClimberMotors(speed);
    }
  }

  public Command lowerClimberArmsCommand(double speed) {
    return Commands.run(() -> lowerClimber(speed));
  }

  public Command raiseClimberArmsCommand(double speed) {
    return Commands.run(() -> raiseClimber(speed));
  }

  @Log.NT(key = "leftRPM")
  public double getRPMLeft() {
    return climberEncoderLeft.getVelocity();
  }

  @Log.NT(key = "rightRPM")
  public double getRPMRight() {
    return climberEncoderRight.getVelocity();
  }

  @Log.NT(key = "leftposition")
  public double getPositionLeft() {
    if (RobotBase.isReal())
      return climberEncoderLeft.getPosition();
    else
      return simpositionleft;
  }

  @Log.NT(key = "rightposition")
  public double getPositionRight() {
    if (RobotBase.isReal())
      return climberEncoderRight.getPosition();
    else
      return simpositionright;
  }

  @Log.NT(key = "leftattarget")
  public boolean getLeftAtTarget(double target) {
    return getPositionLeft() > target;
  }

  @Log.NT(key = "rightattarget")
  public boolean getRightAtTarget(double target) {
    return getPositionRight() > target;
  }

  @Log.NT(key = "climberleftstickyfault")
  public int getLeftStickyFaults() {
    return climberMotorLeft.getStickyFaults();
  }

  @Log.NT(key = "climberrightstickyfault")
  public int getRightStickyFaults() {
    return climberMotorRight.getStickyFaults();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> climberMotorLeft.clearFaults()),
        runOnce(() -> climberMotorRight.clearFaults()));
  }

  @Log.NT(key = "ClimberLeftAmps")
  public double getLeftAmps() {
    return climberMotorLeft.getOutputCurrent();
  }

  @Log.NT(key = "ClimberRightAmps")
  public double getRightAmps() {
    return climberMotorRight.getOutputCurrent();
  }

  public void lockClimber() {
    climberLock.set(1);// Pref.getPref("LockNumber"));
  }

  public Command lockClimberCommand() {
    return Commands.runOnce(() -> lockClimber());
  }

  public void unlockClimber() {
    climberLock.set(0);// Pref.getPref("UnlockNumber"));
  }

  public Command unlockClimberCommand() {
    return Commands.runOnce(() -> unlockClimber());
  }

  @Override
  public void simulationPeriodic() {
    simpositionleft += currentSpeedLeft * .2;
    simpositionright += currentSpeedRight * .2;

  }
}