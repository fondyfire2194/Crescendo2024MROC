// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import java.security.AllPermission;

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
public class AmpAutoCommands {

        public AmpAutoCommands(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf) {
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

        public Command moveShootCenter1_2(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        boolean innerNoteFirst) {
                return Commands.either(
                                new CenterToShoot(cf, pf.pathMaps.get(
                                                amppaths.Center2ToAmpShoot
                                                                .name()),
                                                swerve, true),
                                new CenterToShoot(cf, pf.pathMaps.get(amppaths.Center1ToAmpShoot
                                                .name()),
                                                swerve, true),
                                () -> innerNoteFirst);
        }

        public Command pickupCenter2_1(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpToCenter2
                                                                                .name())),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpToCenter1
                                                                                .name())),
                                                () -> innerNoteFirst),
                                cf.doIntakeDelayed(2));
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

        public Command pickUpNoteAfterShoot(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, boolean innerNoteFirst) {
                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpShootToCenter1
                                                                                .name())),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpShootToCenter2
                                                                                .name())),
                                                () -> innerNoteFirst),
                                cf.doIntake());
        }

        public Command pickUpNoteAfterShootVision(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new PickupUsingVision(cf,
                                                                pf.pathMaps.get(amppaths.AmpToCenter1
                                                                                .name()),
                                                                transfer, intake, swerve, 1.5, 10),
                                                new PickupUsingVision(cf,
                                                                pf.pathMaps.get(amppaths.AmpToCenter2
                                                                                .name()),
                                                                transfer, intake, swerve, 1.5, 10),
                                                () -> innerNoteFirst),
                                cf.doIntake());
        }

        public Command shootbydistance(CommandFactory cf) {
                return Commands.sequence(
                                cf.positionArmRunShooterByDistance(false, true),
                                shoot(cf));
        }

        public Command shoot(CommandFactory cf) {
                return cf.transferNoteToShooterCommand();
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

}