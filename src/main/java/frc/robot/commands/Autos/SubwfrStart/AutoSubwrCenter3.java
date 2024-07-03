// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SubwfrStart;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.commands.Pathplanner.RunPPath;

/** Add your docs here. */
public class AutoSubwrCenter3 extends SequentialCommandGroup {

        public AutoSubwrCenter3(
                        CommandFactory cf,
                        PathFactory pf,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        ArmSubsystem arm,
                        boolean wing3) {

                addCommands(
                                sac.setsbwrstart(swerve, cf, intake),
                                sac.sbwfrShoot(cf),
                                sac.runPathPickupAndShootIfNote(pf.pathMaps.get(sbwfrpaths.SubwfrShootToWing2.name()),
                                                swerve, transfer, arm,
                                                cf, pf, 1),
                                Commands.runOnce(
                                                () -> swerve.pickupTargetX = FieldConstants.FIELD_LENGTH
                                                                / 2),
                                sac.moveAndPickup(sbwfrpaths.Wing2ToCenter3, swerve, cf, pf),
                                Commands.either(
                                                sac.sbwfrmoveandshoot(sbwfrpaths.Center3ToSubwfrShoot,
                                                                swerve, cf,
                                                                pf),
                                                new RunPPath(swerve, pf.pathMaps
                                                                .get(sbwfrpaths.Center3ToSubwfrShoot
                                                                                .name())),
                                                () -> transfer.noteAtIntake()),
                                Commands.runOnce(() -> swerve.pickupTargetX = AllianceUtil
                                                .getWingNoteX()),
                                Commands.either(
                                                sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing3Shoot,
                                                                swerve, cf,
                                                                pf),
                                                sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing1Shoot,
                                                                swerve, cf,
                                                                pf),
                                                () -> wing3),
                                new AutoAlignSpeaker(swerve, 1, true),
                                sac.shootbydistance(cf));
        }

}