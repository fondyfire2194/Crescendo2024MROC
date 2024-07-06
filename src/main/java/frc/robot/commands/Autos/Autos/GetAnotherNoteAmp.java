// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetAnotherNoteAmp extends SequentialCommandGroup {
        /** Creates a new GetAnotherNote. */
        public GetAnotherNoteAmp(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf,
                        PathFactory pf) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());

                addCommands(
                                Commands.sequence(
                                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                                new RotateToAngle(swerve, 90),
                                                Commands.deadline(
                                                                new LookForAnotherNote(swerve, transfer, intake),
                                                                cf.doIntake()),
                                                Commands.waitSeconds(.25),
                                                cf.autopathfind(AllianceUtil.getAmpClearStagePose(),
                                                                SwerveConstants.pfConstraints, 0, 0),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.StageClearToAmpShoot
                                                                                .name())),
                                                Commands.parallel(
                                                                cf.positionArmRunShooterByDistance(
                                                                                true),
                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                cf.transferNoteToShooterCommand()));

        }
}
