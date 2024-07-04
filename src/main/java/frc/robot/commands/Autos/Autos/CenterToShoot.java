// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterToShoot extends SequentialCommandGroup {
       
        public CenterToShoot(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve,
                        boolean source) {

                addCommands(
                                Commands.sequence(
                                                Commands.runOnce(() -> SmartDashboard.putBoolean("GI+OT", true)),
                                                Commands.parallel(
                                                                new RunPPath(swerve, path),
                                                                cf.positionArmRunShooterSpecialCase(
                                                                                Constants.source_ampShootAngle,
                                                                                Constants.source_ampShootSpeed, 15)),
                                                Commands.parallel(
                                                                Commands.runOnce(() -> SmartDashboard
                                                                                .putBoolean("GI+OT3", true)),
                                                                cf.positionArmRunShooterByDistance(false, true),
                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                cf.transferNoteToShooterCommand()));
        }
}
