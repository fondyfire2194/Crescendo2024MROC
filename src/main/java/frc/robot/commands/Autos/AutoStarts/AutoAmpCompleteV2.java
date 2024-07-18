// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.commands.Autos.Autos.GetAnotherNoteAmp;
import frc.robot.commands.Autos.Autos.SourceAmpAutoCommands;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class AutoAmpCompleteV2 extends SequentialCommandGroup {

        public AutoAmpCompleteV2(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SourceAmpAutoCommands srcac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        boolean innerNoteFirst) {

                addCommands(
                                srcac.setAmpStart(),
                                
                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed)),
                                cf.transferNoteToShooterCommand(),

                                srcac.pickupCenter(pf.pathMaps.get(amppaths.AmpToCenter2.name()),
                                                pf.pathMaps.get(amppaths.AmpToCenter1.name()),
                                                innerNoteFirst),
                                // if note in intake go shoot it or try adjacent one

                                Commands.either(
                                                srcac.moveShootCenter(pf.pathMaps.get(amppaths.Center2ToAmpShoot
                                                                                .name()),
                                                                pf.pathMaps.get(amppaths.Center1ToAmpShoot
                                                                                .name()),
                                                                innerNoteFirst),

                                                srcac.pickUpAdjacentNote(pf.pathMaps.get(amppaths.Center2ToCenter1
                                                                                .name()),
                                                                pf.pathMaps.get(amppaths.Center1ToCenter2
                                                                                .name()),
                                                                innerNoteFirst),
                                                () -> cf.noteAtIntake()),

                                // need to know if we shot or tried to pickup adjacent
                                // note
                                // if the latter robot is close to field center

                                Commands.either(
                                                srcac.pickUpNoteAfterShootVision(pf.pathMaps.get(amppaths.Center2ToCenter1
                                                                                .name()),
                                                                pf.pathMaps.get(amppaths.Center1ToCenter2
                                                                                .name()),

                                                                innerNoteFirst),
                                                Commands.none(),
                                                () -> !cf.noteAtIntake()
                                                                && Math.abs(FieldConstants.FIELD_LENGTH
                                                                                / 2
                                                                                - swerve.getX()) > 2),

                                Commands.either(
                                                srcac.moveShootCenter(
                                                                pf.pathMaps.get(amppaths.Center1ToAmpShoot.name()),
                                                                pf.pathMaps.get(amppaths.Center2ToAmpShoot
                                                                                .name()),
                                                                innerNoteFirst),
                                                new GetAnotherNoteAmp(swerve,
                                                                transfer, intake, cf,
                                                                pf),
                                                () -> cf.noteAtIntake()),
                                new RunPPath(swerve, pf.pathMaps.get(amppaths.AmpShootToCenter2.name())));
        }

}