// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SubwfrStart;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShootingData;

/** Add your docs here. */
public class SubwooferAutoCommands {
        private final SwerveSubsystem m_swerve;
        private final IntakeSubsystem m_intake;
        private final ShooterSubsystem m_shooter;
        private final ArmSubsystem m_arm;
        private final TransferSubsystem m_transfer;
        private final CommandFactory m_cf;
        private final PathFactory m_pf;
        private final ShootingData m_sd;

        public SubwooferAutoCommands(SwerveSubsystem swerve, IntakeSubsystem intake,
                        ShooterSubsystem shooter, ArmSubsystem arm, TransferSubsystem transfer,
                        CommandFactory cf, PathFactory pf, ShootingData sd) {
                m_swerve = swerve;
                m_intake = intake;
                m_shooter = shooter;
                m_transfer = transfer;
                m_arm = arm;
                m_cf = cf;
                m_pf = pf;
                m_sd = sd;
        }

        public Command setsbwrstart() {
                return Commands.sequence(
                                Commands.runOnce(() -> m_swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> m_swerve.inhibitVision = true),
                                Commands.runOnce(() -> m_intake.resetIsIntakingSim()),
                                Commands.runOnce(() -> m_transfer.simnoteatintake = true),
                                Commands.runOnce(() -> m_shooter.setRPMTolerancePCT(10)),
                                Commands.runOnce(() -> m_arm.setTolerance(Units.degreesToRadians(1))),
                                m_cf.setStartPosebyAlliance(FieldConstants.sbwfrStartPose));
        }

        public Command shoot() {
                return m_cf.transferNoteToShooterCommand();
        }

        public Command setArmShooter(double angle, double rpm) {
                return m_cf.positionArmRunShooterSpecialCase(angle, rpm);
        }

        public Command shoot(double angle, double rpm) {
                return Commands.sequence(
                                setArmShooter(angle, rpm),
                                m_cf.checkAtTargets(20),
                                shoot());
        }

        public Command sbwfrShoot() {
                return Commands.sequence(
                                setArmShooter(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                m_cf.checkAtTargets(20),
                                shoot());
        }

        public Command sbwfrShootEarly() {
                return Commands.sequence(
                                Commands.race(
                                                Commands.waitSeconds(.5),
                                                setArmShooter(Constants.subwfrArmAngle - Units.degreesToRadians(2),
                                                                Constants.subwfrShooterSpeed + 300)),
                                shoot());
        }

        public Command shootbydistance() {
                return Commands.sequence(
                                m_sd.setArmOffsetDegreesCommand(2),
                                m_cf.positionArmRunShooterByDistance(true),
                                shoot());
        }

        public Command move(sbwfrpaths path) {
                return new RunPPath(m_swerve, m_pf.pathMaps.get(path.name()));
        }

        public Command moveandshoot(sbwfrpaths path,
                        double angle, double rpm) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(m_swerve, m_pf.pathMaps.get(path.name())),
                                                setArmShooter(angle, rpm)),
                                Commands.waitUntil(() -> m_shooter.bothAtSpeed() && m_arm.getAtSetpoint()),
                                shoot());
        }

        public Command sbwfrmoveandshoot(sbwfrpaths path) {
                return Commands.sequence(
                                Commands.parallel(
                                                move(path),
                                                setArmShooter(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed)),
                                shoot());
        }

        public Command moveAndPickup(sbwfrpaths path) {
                return Commands.parallel(
                                move(path),
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

}
