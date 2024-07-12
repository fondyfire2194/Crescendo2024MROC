// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.commands.Autos.SubwfrStart.SubwooferAutoCommands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.ShootingData;

/** Add your docs here. */
public class AutoSubwr5Note extends SequentialCommandGroup {

        public AutoSubwr5Note(
                        CommandFactory cf,
                        PathFactory pf,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ArmSubsystem arm,
                        ShootingData sd) {

                addCommands( // note
                                sac.setsbwrstart(),
                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed)),
                                cf.transferNoteToShooterCommand(),

                                sac.runPathPickupAndShootIfNote(pf.pathMaps.get(sbwfrpaths.SubToNote3Fast.name()),
                                                1),

                                sac.runPathPickupAndShootIfNote(pf.pathMaps.get(sbwfrpaths.Note3ToNote2Fast.name()),
                                                1),

                                sac.runPathPickupAndShootIfNote(pf.pathMaps.get(sbwfrpaths.Note2ToNote1Fast.name()),
                                                1),

                                Commands.parallel(
                                                new PickupUsingVision(cf,
                                                                pf.pathMaps.get(sbwfrpaths.Note1ToCenter1Fast
                                                                                .name()),
                                                                null, intake, swerve, 2.0, 10),
                                                cf.doIntake()),

                                new CenterToShoot(cf,
                                                pf.pathMaps.get(sbwfrpaths.Center1ToAmpShoot
                                                                .name()),
                                                swerve, sd, true));
        }

}