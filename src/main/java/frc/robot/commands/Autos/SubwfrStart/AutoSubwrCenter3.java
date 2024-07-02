// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SubwfrStart;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Autos.Autos.GetAnotherNoteAmp;
import frc.robot.commands.Autos.Autos.GetAnotherNoteSource;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

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
                        boolean wing3,
                        boolean usesourceside) {

                addCommands( // note

                                Commands.sequence(
                                                sac.setsbwrstart(swerve, cf),
                                                sac.sbwfrShoot(cf),
                                                sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing2, swerve, cf,
                                                                pf),
                                                sac.shootbydistance(cf),
                                                sac.moveAndPickup(sbwfrpaths.Wing2ToCenter3, swerve, cf, pf),
                                                Commands.either(
                                                                sac.sbwfrmoveandshoot(sbwfrpaths.Center3ToSubwfrShoot,
                                                                                swerve, cf,
                                                                                pf),
                                                                Commands.either(
                                                                                new GetAnotherNoteSource(swerve,
                                                                                                transfer, intake,
                                                                                                cf, pf),
                                                                                new GetAnotherNoteAmp(swerve, transfer,
                                                                                                intake,
                                                                                                cf, pf),
                                                                                () -> usesourceside),
                                                                () -> transfer.noteAtIntake()),
                                                Commands.either(
                                                                sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing3Shoot,
                                                                                swerve, cf,
                                                                                pf),
                                                                sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing1Shoot,
                                                                                swerve, cf,
                                                                                pf),
                                                                () -> wing3),
                                                new AutoAlignSpeaker(swerve, 1, true),
                                                sac.shootbydistance(cf)));
        }

}