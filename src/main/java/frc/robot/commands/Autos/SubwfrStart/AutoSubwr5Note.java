// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SubwfrStart;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.LLPipelines;

/** Add your docs here. */
public class AutoSubwr5Note extends SequentialCommandGroup {

        public AutoSubwr5Note(
                        CommandFactory cf,
                        PathFactory pf,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        ArmSubsystem arm) {

                addCommands( // note
                                sac.setsbwrstart(swerve, cf),
                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 20)),
                                cf.transferNoteToShooterCommand(),

                                sac.runPathPickupAndShoot(pf.pathMaps.get(sbwfrpaths.SubToNote3Fast.name()), swerve,
                                                arm,
                                                cf, pf),

                                sac.runPathPickupAndShoot(pf.pathMaps.get(sbwfrpaths.Note3ToNote2Fast.name()), swerve,
                                                arm,
                                                cf, pf),

                                sac.runPathPickupAndShoot(pf.pathMaps.get(sbwfrpaths.Note2ToNote1Fast.name()), swerve,
                                                arm,
                                                cf, pf),

                                Commands.parallel(
                                                new PickupUsingVision(cf,
                                                                pf.pathMaps.get(sbwfrpaths.Note1ToCenter1Fast
                                                                                .name()),
                                                                transfer, intake, swerve, 2.0, 10,
                                                                LLPipelines.pipelines.NOTEDET1.ordinal()),

                                                cf.doIntake()),

                                new CenterToShoot(cf,
                                                pf.pathMaps.get(sbwfrpaths.Center1ToShootFast
                                                                .name()),
                                                swerve, true));
        }


}