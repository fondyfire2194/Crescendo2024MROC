// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Autos.SubwfrStart.SubwooferAutoCommands;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
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
                        boolean wing3) {

                addCommands(
                                sac.setsbwrstart(swerve, cf, intake),
                                sac.sbwfrShoot(cf),
                                sac.runPathPickupAndShootIfNote(pf.pathMaps.get(sbwfrpaths.SubwfrShootToWing2.name()),
                                                swerve, cf, pf, 1),
                                sac.moveAndPickup(sbwfrpaths.Wing2ToCenter3, swerve, cf, pf),
                                Commands.either(
                                                sac.sbwfrmoveandshoot(sbwfrpaths.Center3ToSubwfrShoot,
                                                                swerve, cf,
                                                                pf),
                                                new RunPPath(swerve, pf.pathMaps
                                                                .get(sbwfrpaths.Center3ToSubwfrShoot
                                                                                .name())),
                                                () -> cf.noteAtIntake()),
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