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
import frc.robot.utils.ShootingData;

/** Add your docs here. */
public class SourceAmpAutoCommands {
        private final SwerveSubsystem m_swerve;
        private final TransferSubsystem m_transfer;
        private final IntakeSubsystem m_intake;
        private final CommandFactory m_cf;
        private final PathFactory m_pf;
        private final ShootingData m_sd;

        public SourceAmpAutoCommands(SwerveSubsystem swerve, IntakeSubsystem intake, TransferSubsystem transfer,
                        CommandFactory cf, PathFactory pf, ShootingData sd) {
                m_swerve = swerve;
                m_intake = intake;
                m_transfer = transfer;
                m_cf = cf;
                m_pf = pf;
                m_sd=sd;
        }

        public Command setSourceStart() {
                return Commands.sequence(
                                Commands.runOnce(() -> m_transfer.simnoteatintake = true),
                                Commands.runOnce(() -> m_intake.resetIsIntakingSim()),
                                Commands.runOnce(() -> m_swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> m_swerve.ampActive = false),
                                Commands.runOnce(() -> m_swerve.sourceActive = true),
                                Commands.runOnce(() -> m_swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                m_cf.setStartPosebyAlliance(FieldConstants.sourceStartPose));

        }

        public Command setAmpStart() {
                return Commands.sequence(
                                Commands.runOnce(() -> m_transfer.simnoteatintake = false),
                                Commands.runOnce(() -> m_intake.resetIsIntakingSim()),
                                Commands.runOnce(() -> m_swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> m_swerve.ampActive = true),
                                Commands.runOnce(() -> m_swerve.sourceActive = false),
                                Commands.runOnce(() -> m_swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                m_cf.setStartPosebyAlliance(FieldConstants.ampStartPose));

        }

        public Command pickupNote(PathPlannerPath path) {

                return Commands.parallel(
                                new RunPPath(m_swerve, path),
                                m_cf.doIntake());

        }

        public Command moveShoot(PathPlannerPath path, double armAngle,
                        double shooterpm, double rpmtol) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(m_swerve, path),
                                                m_cf.positionArmRunShooterSpecialCase(armAngle, shooterpm)),
                                m_cf.transferNoteToShooterCommand());
        }

        public Command pickupCenter(
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(m_swerve,
                                                                path1),
                                                new RunPPath(m_swerve,
                                                                path2),
                                                () -> innerNoteFirst),
                                m_cf.doIntakeDelayed(2));
        }

        public Command moveShootCenter(
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {
                return Commands.either(
                                new CenterToShoot(m_cf, path1,
                                                m_swerve, m_sd, true),
                                new CenterToShoot(m_cf, path2,
                                                m_swerve, m_sd, true),
                                () -> innerNoteFirst);
        }

        public Command pickUpNoteAfterShoot(
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(m_swerve,
                                                                path1),
                                                new RunPPath(m_swerve,
                                                                path2),
                                                () -> innerNoteFirst),
                                m_cf.doIntake());
        }

        public Command pickUpAdjacentNote(
                        PathPlannerPath path1,
                        PathPlannerPath path2, boolean innerNoteFirst) {
                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(m_swerve,
                                                                path1),
                                                new RunPPath(m_swerve,
                                                                path2),
                                                () -> innerNoteFirst),
                                m_cf.doIntake());
        }

        public Command pickUpNoteAfterShootVision(
                        PathPlannerPath path1, PathPlannerPath path2,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new PickupUsingVision(m_cf,
                                                                path1,
                                                                m_transfer, m_intake, m_swerve, 1.5, 10),
                                                new PickupUsingVision(m_cf,
                                                                path2,
                                                                m_transfer, m_intake, m_swerve, 1.5, 10),
                                                () -> innerNoteFirst),
                                m_cf.doIntake());
        }

        public Command runPathPickupAndShootIfNote(PathPlannerPath path, double aligntolerance) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(m_swerve, path),
                                                m_cf.doIntake()),
                                Commands.either(
                                                Commands.sequence(
                                                                Commands.parallel(
                                                                                m_cf.positionArmRunShooterByDistance(
                                                                                                true),
                                                                                new AutoAlignSpeaker(m_swerve,
                                                                                                aligntolerance, true)),
                                                                m_cf.transferNoteToShooterCommand()),
                                                Commands.none(),
                                                () -> m_cf.noteAtIntake()));
        }

        public Command pickupCenter2_1FromWing1(
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(m_swerve,
                                                                m_pf.pathMaps.get(amppaths.Wing1ToCenter2
                                                                                .name())),
                                                new RunPPath(m_swerve,
                                                                m_pf.pathMaps.get(amppaths.Wing1ToCenter1
                                                                                .name())),
                                                () -> innerNoteFirst),
                                m_cf.doIntakeDelayed(2));
        }

}