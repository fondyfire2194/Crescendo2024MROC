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
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.Autos.GetAnotherNoteSource;
import frc.robot.commands.Autos.Autos.SourceAmpAutoCommands;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class AutoSourceCompleteV2 extends SequentialCommandGroup {

        public AutoSourceCompleteV2(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SourceAmpAutoCommands srcac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        boolean innerNoteFirst) {

                addCommands( 
                                srcac.setSourceStart(),
                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed)),
                                cf.transferNoteToShooterCommand(),

                                srcac.pickupCenter(pf.pathMaps.get(sourcepaths.SourceToCenter4.name()),
                                                pf.pathMaps.get(sourcepaths.SourceToCenter5.name()),
                                                innerNoteFirst),
                                // if note in intake go shoot it or try adjacent one

                                Commands.either(
                                                srcac.moveShootCenter(pf.pathMaps.get(sourcepaths.Center4ToSourceShoot
                                                                                .name()),
                                                                pf.pathMaps.get(sourcepaths.Center5ToSourceShoot
                                                                                .name()),
                                                                innerNoteFirst),

                                                srcac.pickUpAdjacentNote(pf.pathMaps.get(sourcepaths.Center4ToCenter5
                                                                                .name()),
                                                                pf.pathMaps.get(sourcepaths.Center5ToCenter4
                                                                                .name()),
                                                                innerNoteFirst),
                                                () -> cf.noteAtIntake()),

                                // need to know if we shot or tried to pickup adjacent
                                // note
                                // if the latter robot is close to field center

                                Commands.either(
                                                srcac.pickUpNoteAfterShootVision(pf.pathMaps.get(sourcepaths.SourceShootToCenter5
                                                                                .name()),
                                                                pf.pathMaps.get(sourcepaths.SourceShootToCenter4
                                                                                .name()),
                                                                innerNoteFirst),
                                                Commands.none(),
                                                () -> !cf.noteAtIntake()
                                                                && Math.abs(FieldConstants.FIELD_LENGTH
                                                                                / 2
                                                                                - swerve.getX()) > 2),

                                Commands.either(
                                                srcac.moveShootCenter(
                                                                pf.pathMaps.get(sourcepaths.Center5ToSourceShoot
                                                                                .name()),
                                                                pf.pathMaps.get(sourcepaths.Center4ToSourceShoot
                                                                                .name()),
                                                                innerNoteFirst),
                                                new GetAnotherNoteSource(swerve,
                                                                transfer, intake, cf,
                                                                pf),
                                                () -> cf.noteAtIntake()),
                                new RunPPath(swerve, pf.pathMaps.get(sourcepaths.SourceShootToCenter4.name())));
        }

}