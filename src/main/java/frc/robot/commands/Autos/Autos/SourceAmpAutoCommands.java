// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class SourceAmpAutoCommands {

        public SourceAmpAutoCommands(SwerveSubsystem swerve, CommandFactory cf) {
        }

        public Command setSourceStart(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf) {
                return Commands.sequence(
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                Commands.runOnce(() -> intake.resetIsIntakingSim()),
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = false),
                                Commands.runOnce(() -> swerve.sourceActive = true),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose));

        }

        public Command setAmpStart(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf) {
                return Commands.sequence(
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                Commands.runOnce(() -> intake.resetIsIntakingSim()),
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = true),
                                Commands.runOnce(() -> swerve.sourceActive = false),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose));

        }

        public Command pickupNote(CommandFactory cf, PathPlannerPath path, SwerveSubsystem swerve) {

                return Commands.parallel(
                                new RunPPath(swerve, path),
                                cf.doIntake());

        }

        public Command moveShoot(CommandFactory cf, PathPlannerPath path, SwerveSubsystem swerve, double armAngle,
                        double shooterpm, double rpmtol) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(swerve, path),
                                                cf.positionArmRunShooterSpecialCase(armAngle, shooterpm, rpmtol)),
                                cf.transferNoteToShooterCommand());
        }

        public Command pickupCenter(CommandFactory cf, SwerveSubsystem swerve,
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                path1),
                                                new RunPPath(swerve,
                                                                path2),
                                                () -> innerNoteFirst),
                                cf.doIntakeDelayed(2));
        }

        public Command moveShootCenter(CommandFactory cf, SwerveSubsystem swerve,
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {
                return Commands.either(
                                new CenterToShoot(cf, path1,
                                                swerve, true),
                                new CenterToShoot(cf, path2,
                                                swerve, true),
                                () -> innerNoteFirst);
        }

        public Command pickUpNoteAfterShoot(CommandFactory cf, SwerveSubsystem swerve,
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                path1),
                                                new RunPPath(swerve,
                                                                path2),
                                                () -> innerNoteFirst),
                                cf.doIntake());
        }

        public Command pickUpAdjacentNote(CommandFactory cf, SwerveSubsystem swerve,
                        PathPlannerPath path1,
                        PathPlannerPath path2, boolean innerNoteFirst) {
                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                path1),
                                                new RunPPath(swerve,
                                                                path2),
                                                () -> innerNoteFirst),
                                cf.doIntake());
        }

        public Command pickUpNoteAfterShootVision(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake,
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new PickupUsingVision(cf,
                                                                path1,
                                                                transfer, intake, swerve, 1.5, 10),
                                                new PickupUsingVision(cf,
                                                                path2,
                                                                transfer, intake, swerve, 1.5, 10),
                                                () -> innerNoteFirst),
                                cf.doIntake());
        }

        public Command runPathPickupAndShootIfNote(PathPlannerPath path, SwerveSubsystem swerve,
                        CommandFactory cf, PathFactory pf, double aligntolerance) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(swerve, path),
                                                cf.doIntake()),
                                Commands.either(
                                                Commands.sequence(
                                                                Commands.parallel(
                                                                                cf.positionArmRunShooterByDistance(
                                                                                                false, true),
                                                                                new AutoAlignSpeaker(swerve,
                                                                                                aligntolerance, true)),
                                                                cf.transferNoteToShooterCommand()),
                                                Commands.none(),
                                                () -> cf.noteAtIntake()));
        }

        public Command pickupCenter2_1FromWing1(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.Wing1ToCenter2
                                                                                .name())),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.Wing1ToCenter1
                                                                                .name())),
                                                () -> innerNoteFirst),
                                cf.doIntakeDelayed(2));
        }

}