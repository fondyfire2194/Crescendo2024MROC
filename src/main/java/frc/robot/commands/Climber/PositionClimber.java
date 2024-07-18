// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class PositionClimber extends Command {
  /** Creates a new JogShooter. */
  private ClimberSubsystem m_climber;
  private double m_target;
  private double m_speed;

  public PositionClimber(ClimberSubsystem climber, double target, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_target = target;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double positionError = m_target - m_climber.getPositionLeft();
    double usespeed = m_speed;

    if (positionError < 6)
      usespeed = m_speed / 2;

    if (positionError < 1)
      usespeed = m_speed / 4;


    if (!m_climber.getLeftAtTarget(m_target)) {
      m_climber.runLeftClimberMotor(usespeed);
      m_climber.runRightClimberMotor(usespeed);
    } else {
      m_climber.stopLeftMotor();
      m_climber.stopRightMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.getLeftAtTarget(m_target);
  }
}
