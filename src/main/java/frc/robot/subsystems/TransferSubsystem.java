// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.Pref;
import monologue.Annotations.Log;
import monologue.Logged;

public class TransferSubsystem extends SubsystemBase implements Logged {

  public CANSparkMax transferMotor;
  public SparkPIDController transferController;
  RelativeEncoder transferEncoder;

  @Log.NT(key = "transfercommandrpm")
  private double commandrpm;

  public SparkLimitSwitch m_limitSwitch;
  @Log.NT(key = "simnoteatintake")
  public boolean simnoteatintake;
  public boolean skipFirstNoteInSim;
  public boolean skipSecondNoteInSim;
  public boolean skipThirdNoteInSim;
  public boolean skipFourthNoteInSim;

  @Log.NT(key = "lobbing")
  public boolean lobbing;
  @Log.NT(key = "shootmoving")
  public boolean shootmoving;
  public boolean transferMotorConnected;
  @Log.NT(key = "okshootmoving")
  public boolean OKShootMoving;

  Debouncer noteDetector = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  /** Creates a new transfer. */
  public TransferSubsystem() {

    transferMotor = new CANSparkMax(CANIDConstants.transferID, MotorType.kBrushless);
    transferEncoder = transferMotor.getEncoder();
    transferController = transferMotor.getPIDController();

    configMotor(transferMotor, transferEncoder, transferController, true);

    m_limitSwitch = transferMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_limitSwitch.enableLimitSwitch(true);
  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(TransferConstants.transferContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(TransferConstants.transferIdleMode);
    // encoder.setVelocityConversionFactor(TransferConstants.transferConversionVelocityFactor);
    // encoder.setPositionConversionFactor(TransferConstants.transferConversionPositionFactor);
    motor.enableVoltageCompensation(TransferConstants.voltageComp);
    motor.setClosedLoopRampRate(.1);
    int i = 0;
    int loop = 100;
    while (i < loop) {
      i++;
    }
    controller.setFF(TransferConstants.transferKFF);
    controller.setP(TransferConstants.transferKp);
    controller.setI(TransferConstants.transferKi);
    controller.setD(TransferConstants.transferKd);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void stopMotor() {
    runAtVelocity(0);
    transferMotor.stopMotor();
    commandrpm = 0;
  }

  public Command stopTransferCommand() {
    commandrpm = 0;
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public Command transferToShooterCommand() {
    return Commands.run(() -> transferToShooter()).until(() -> !noteAtIntake() && RobotBase.isReal())
        .withTimeout(TransferConstants.clearShooterTime)
        .andThen(stopTransferCommand());
  }

  public Command transferToShooterCommandAmp() {
    return Commands.run(() -> transferToShooterAmp())
        .until(() -> !noteAtIntake())
        .withTimeout(TransferConstants.clearShooterTime)
        .andThen(stopTransferCommand());
  }

  public void transferToShooterAmp() {
    enableLimitSwitch(false);
    commandrpm = Pref.getPref("AmpTransferToShootSpeed");
    simnoteatintake = false;
    runAtVelocity(commandrpm);
  }

  public void transferToShooter() {
    enableLimitSwitch(false);
    commandrpm = TransferConstants.transferToShootSpeed;// Pref.getPref("TransferToShootSpeed");
    runAtVelocity(commandrpm);
    simnoteatintake = false;
  }

  public void runToSensor() {
    enableLimitSwitch(true);

    // commandrpm=Pref.getPref("TransferIntakingSpeed");
    commandrpm = TransferConstants.intakingSpeed;
    runAtVelocity(commandrpm);
  }

  public void runAtVelocity(double rpm) {
    if (RobotBase.isReal())
      transferController.setReference(rpm, ControlType.kVelocity);
  }

  @Log.NT(key = "transfernoteatintake")
  public boolean noteAtIntake() {
    return m_limitSwitch.isPressed();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!transferMotorConnected) {
      transferMotorConnected = checkMotorCanOK(transferMotor);
      SmartDashboard.putBoolean("Transfer//OKTransferMotor", transferMotorConnected);
    }
  }

  private boolean checkMotorCanOK(CANSparkMax motor) {
    double temp = motor.getOpenLoopRampRate();
    return RobotBase.isSimulation() || motor.setOpenLoopRampRate(temp) == REVLibError.kOk;
  }

  public Command testCan() {
    return Commands.runOnce(() -> transferMotorConnected = false);
  }

  public void enableLimitSwitch(boolean enable) {
    m_limitSwitch.enableLimitSwitch(enable);
  }

  public boolean getLimitSwitchEnabled() {
    return m_limitSwitch.isLimitSwitchEnabled();
  }

  @Log.NT(key = "transferamps")
  public double getAmps() {
    return transferMotor.getOutputCurrent();
  }

  @Log.NT(key = "transferrpm")
  public double getRPM() {
    if (RobotBase.isReal())
      return transferEncoder.getVelocity();
    else
      return commandrpm;
  }

  public boolean onPlusHardwareLimit() {
    return transferMotor.getFault(FaultID.kHardLimitRev);
  }

  public boolean onMinusHardwareLimit() {
    return transferMotor.getFault(FaultID.kHardLimitRev);
  }

  @Log.NT(key = "transferstickyfault")
  public int getStickyFaults() {
    return transferMotor.getStickyFaults();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> transferMotor.clearFaults());
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public double getPosition() {
    return transferEncoder.getPosition();
  }

  public double getVelocity() {
    return transferEncoder.getVelocity();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < Pref.getPref("TransferIntakingSpeed") / 20;
  }

}
