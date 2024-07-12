// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class IntakeSubsystem extends SubsystemBase implements Logged {

  public CANSparkMax intakeMotor;
  RelativeEncoder intakeEncoder;
  public SparkPIDController intakeController;

  @Log.NT(key = "intakerun")
  private boolean runIntake;
  private boolean reverseIntake;
  public boolean jogging;

  @Log.NT(key = "intakecommandrpm")
  private double commandrpm;
  @Log.NT(key = "notemissed")
  public boolean noteMissed;
  public boolean intakeMotorConnected;
  @Log.NT(key = "isIntaking1")
  public boolean isIntaking1;
  @Log.NT(key = "isIntaking2")
  public boolean isIntaking2;
  @Log.NT(key = "isIntaking3")
  public boolean isIntaking3;
  @Log.NT(key = "isIntaking4")
  public boolean isIntaking4;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.CANIDConstants.intakeID, MotorType.kBrushless);
    intakeController = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    configMotor(intakeMotor, intakeEncoder, intakeController, false);
  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.IntakeConstants.intakeContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.IntakeConstants.intakeIdleMode);
    encoder.setVelocityConversionFactor(Constants.IntakeConstants.intakeConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.IntakeConstants.intakeConversionPositionFactor);
    motor.enableVoltageCompensation(Constants.IntakeConstants.voltageComp);
    motor.setClosedLoopRampRate(2);
    int i = 0;
    int loop = 100;
    while (i < loop) {
      i++;
    }
    controller.setFF(IntakeConstants.intakeKFF);
    controller.setP(IntakeConstants.intakeKp);
    controller.setI(IntakeConstants.intakeKi);
    controller.setD(IntakeConstants.intakeKd);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void stopMotor() {
    intakeMotor.stopMotor();
    intakeController.setReference(0, ControlType.kVelocity);
    resetRunIntake();
    resetReverseIntake();
    commandrpm = 0;
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopMotor());
  }

  public Command startIntakeCommand() {
    return Commands.runOnce(() -> setRunIntake());
  }

  public Command reverseIntakeCommand() {
    return Commands.runOnce(() -> setReverseIntake());
  }

  public void setRunIntake() {
    runIntake = true;
    reverseIntake = false;
  }

  public void resetRunIntake() {
    runIntake = false;
  }

  public boolean getRunIntake() {
    return runIntake;
  }

  public void setReverseIntake() {
    reverseIntake = true;
    runIntake = false;
  }

  public void resetReverseIntake() {
    reverseIntake = false;
  }

  public boolean getReverseIntake() {
    return reverseIntake;
  }

  public void resetIsIntakingSim() {
    isIntaking1 = false;
    isIntaking2 = false;
    isIntaking3 = false;
    isIntaking4 = false;
  }

  @Log.NT(key = "intakerpm")
  public double getRPM() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (runIntake && !reverseIntake) {
      commandrpm = IntakeConstants.intakeSpeed;// Pref.getPref("IntakeSpeed");
      runAtVelocity(commandrpm);
    }

    if (reverseIntake && !runIntake) {
      commandrpm = -IntakeConstants.intakeSpeed;// Pref.getPref("IntakeSpeed");
      runAtVelocity(commandrpm);
    }

    if (!runIntake && !reverseIntake && !jogging) {
      stopMotor();
    }

    if (!intakeMotorConnected) {
      intakeMotorConnected = checkMotorCanOK(intakeMotor);
      SmartDashboard.putBoolean("Intake//OKIntakeMotor", intakeMotorConnected);
    }
  }

  private boolean checkMotorCanOK(CANSparkMax motor) {
    double temp = motor.getOpenLoopRampRate();
    return RobotBase.isSimulation() || motor.setOpenLoopRampRate(temp) == REVLibError.kOk;
  }

  public Command testCan() {
    return runOnce(() -> intakeMotorConnected = false);
  }

  private void runAtVelocity(double rpm) {
    intakeController.setReference(rpm, ControlType.kVelocity);
  }

  public void reverseMotor() {
    runAtVelocity(IntakeConstants.reverseRPM);
  }

  @Log.NT(key = "intakeAmps")
  public double getAmps() {
    return intakeMotor.getOutputCurrent();
  }

  public void setPID() {
    intakeController.setP(IntakeConstants.intakeKp);// Pref.getPref("IntakeKp"));
    intakeController.setFF(IntakeConstants.intakeKFF);
  }

  @Log.NT(key = "intakestickyfault")
  public int getStickyFaults() {
    return intakeMotor.getStickyFaults();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> intakeMotor.clearFaults());
  }

}
