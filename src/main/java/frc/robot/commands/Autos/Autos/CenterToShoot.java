// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterToShoot extends SequentialCommandGroup {
        double activeArmAngle = 0;
        double activeShooterRPM = 0;

        public CenterToShoot(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve,
                        boolean source) {

                addCommands(
                                Commands.sequence(
                                                Commands.either(
                                                                Commands.parallel(
                                                                                Commands.runOnce(
                                                                                                () -> activeArmAngle = Constants.sourceShootAngle),
                                                                                Commands.runOnce(
                                                                                                () -> activeShooterRPM = Constants.sourceShootSpeed)),

                                                                Commands.parallel(
                                                                                Commands.runOnce(
                                                                                                () -> activeArmAngle = Constants.ampShootAngle),
                                                                                Commands.runOnce(
                                                                                                () -> activeShooterRPM = Constants.ampShootSpeed)),

                                                                () -> source),
                                                Commands.parallel(
                                                                new RunPPath(swerve, path),
                                                                cf.positionArmRunShooterSpecialCase(activeArmAngle, activeShooterRPM, 15)),
                                                Commands.parallel(
                                                                cf.positionArmRunShooterByDistance(false, true),
                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                cf.transferNoteToShooterCommand()));
        }
}
