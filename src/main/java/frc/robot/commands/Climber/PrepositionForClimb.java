// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepositionForClimb extends SequentialCommandGroup {
  /** Creates a new PrepositionForClimb. */
  int temp = 0;

  public PrepositionForClimb(SwerveSubsystem swerve, CommandFactory cf, boolean away) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        getLoc(swerve,away),

        new DeferredCommand(() -> midampsource(swerve, cf), m_requirements),
        new DeferredCommand(() -> cf.autopathfind(swerve.targetPose, 0, 0), m_requirements));

  }

  private Command midampsource(SwerveSubsystem swerve, CommandFactory cf) {

    switch (temp) {
      case 0:
        return setTargetPose(swerve, AllianceUtil.getAlliancePose(FieldConstants.stageBlueAllianceClimb));
      case 1:
        return setTargetPose(swerve, FieldConstants.blueAmpCenter);
      case 2:
        return setTargetPose(swerve, FieldConstants.blueSourceCenter);
      case 5:
        return setTargetPose(swerve, AllianceUtil.getAlliancePose(FieldConstants.stageBlueAllianceClimb));
      case 6:
        return setTargetPose(swerve, FieldConstants.blueAmpMidfield);
      case 7:
        return setTargetPose(swerve, FieldConstants.blueSourceMidfield);
      case 10:
        return setTargetPose(swerve, AllianceUtil.getAlliancePose(FieldConstants.stageBlueAllianceClimb));
      case 11:
        return setTargetPose(swerve, FieldConstants.redAmpCenter);
      case 12:
        return setTargetPose(swerve, FieldConstants.redSourceCenter);
      case 15:
        return setTargetPose(swerve, AllianceUtil.getAlliancePose(FieldConstants.stageBlueAllianceClimb));
      case 16:
        return setTargetPose(swerve, FieldConstants.redAmpMidfield);
      case 17:
        return setTargetPose(swerve, FieldConstants.redSourceMidfield);

      default:
        return Commands.none();

    }

  }

  private Command setTargetPose(SwerveSubsystem swerve, Pose2d pose) {
    return Commands.runOnce(() -> swerve.targetPose = pose);
  }

  private Command getLoc(SwerveSubsystem swerve, boolean away) {

    return new FunctionalCommand(

        () -> Commands.none(),

        () -> {
          temp = 0;
          if (swerve.getY() > FieldConstants.FIELD_WIDTH / 2 + 2)
            temp = 1;
          if (swerve.getY() < FieldConstants.FIELD_WIDTH / 2 - 2)
            temp = 2;
          if (away)
            temp += 5;
          Optional<Alliance> ally = DriverStation.getAlliance();
          if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
              temp += 10;
            }
          }
        },
        (interrupted) -> Commands.none(),
        () -> true);

  }

  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red;
    } else {
      return false;
    }

  }
}
