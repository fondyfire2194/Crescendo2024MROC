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
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoSbwfrShootThenSequence extends SequentialCommandGroup {

        public AutoSbwfrShootThenSequence(
                        CommandFactory cf,
                        PathFactory pf,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ArmSubsystem arm,
                        sbwfrpaths path1,
                        sbwfrpaths path2,
                        sbwfrpaths path3,
                        sbwfrpaths path4) {

                addCommands(

                                sac.setsbwrstart(),

                                sac.sbwfrShoot(),

                                sac.moveAndPickup(path1),

                                Commands.either(
                                                sac.sbwfrmoveandshoot(path2),
                                                new RunPPath(swerve, pf.pathMaps.get(path2.name())),
                                                () -> cf.noteAtIntake()),

                                sac.moveAndPickup(path3),

                                Commands.either(
                                                sac.sbwfrmoveandshoot(path4),
                                                Commands.none(),
                                                () -> cf.noteAtIntake()),

                                cf.resetAll());

        }

        public AutoSbwfrShootThenSequence(
                        CommandFactory cf,
                        PathFactory pf,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ArmSubsystem arm,
                        sbwfrpaths path1,
                        sbwfrpaths path2,
                        sbwfrpaths path3,
                        sbwfrpaths path4,
                        sbwfrpaths path5,
                        sbwfrpaths path6) {

                addCommands(

                                sac.setsbwrstart(),

                                sac.sbwfrShoot(),

                                sac.moveAndPickup(path1),

                                Commands.either(
                                                sac.sbwfrmoveandshoot(path2),
                                                new RunPPath(swerve, pf.pathMaps.get(path2.name())),
                                                () -> cf.noteAtIntake()),

                                sac.moveAndPickup(path3),

                                Commands.either(
                                                sac.sbwfrmoveandshoot(path4),
                                                new RunPPath(swerve, pf.pathMaps.get(path4.name())),
                                                () -> cf.noteAtIntake()),

                                sac.moveAndPickup(path5),

                                Commands.either(
                                                sac.sbwfrmoveandshoot(path6),
                                                Commands.none(),
                                                () -> cf.noteAtIntake()),

                                cf.resetAll());

        }

}
