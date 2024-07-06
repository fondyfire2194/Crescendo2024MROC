// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Factories.CommandFactory;
import frc.robot.subsystems.ArmSubsystem;

public class ViewArmShooterByDistance extends Command {
  /** Creates a new ArmShooterByDistance. */
  private final CommandFactory m_cf;
  private final ShootingData m_sd;
  private final ArmSubsystem m_arm;
  // 78 to 83 less shooter pivot height 10
  private double shooterspeakerheightdifference = Units.inchesToMeters(70);
  private double speakeropeningwidth = Units.inchesToMeters(42);
  private double notediameter = Units.inchesToMeters(14);
  double distance;
  int loopctr;
  boolean endit;
  double deg;
  double distcomp = 0;

  public ViewArmShooterByDistance(CommandFactory cf, ShootingData sd, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cf = cf;
    m_sd = sd;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = 0;
    loopctr = 0;
    endit = false;
    deg = 60;
    distcomp = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;
    if (loopctr == 5 && distance <= 7) {
      double rpm = m_sd.shooterRPMMap.get(distance);
      double angleDeg = Units.radiansToDegrees(m_sd.armAngleMap.get(distance));
      double shotTimeMs = m_sd.shotTimeMap.get(distance) * 1000;
      double toleranceDeg = Units.radiansToDegrees(m_sd.armToleranceMap.get(distance));
      double toleranceRPM = m_sd.shooterRPMToleranceMap.get(distance);
      double theoreticalAngle = Units.radiansToDegrees(Math.atan(shooterspeakerheightdifference / distance));
      
      if (distance > 1 && distance < 4)
        rpm = 3000 + (500 * distance / 4);

      double angleTan = Math.tan(Units.degreesToRadians(angleDeg));

      double aimHeightFromTable = angleTan * distance;

      SmartDashboard.putNumber("ArmCalc/DistRPM", rpm / 100);
      SmartDashboard.putNumber("ArmCalc/DistAngle", angleDeg);
      SmartDashboard.putNumber("ArmCalc/ShotTime", shotTimeMs);
      SmartDashboard.putNumber("ArmCalc/ToleranceAngle", toleranceDeg);


      SmartDashboard.putNumber("ArmCalc/AimHeightTable", aimHeightFromTable);

      SmartDashboard.putNumber("ArmCalc/DistDist", distance);

      SmartDashboard.putNumber("ArmCalc/ToleranceRPM", toleranceRPM);
      SmartDashboard.putNumber("ArmCalc/TheoreticalAngle", theoreticalAngle);

      loopctr = 0;
      distance += .01;
    }

    endit = distance >= 7;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endit;// Units.feetToMeters(19.25);
  }
}
