// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.commands.Autos.Autos.AmpAutoCommands;
import frc.robot.commands.Autos.Autos.GetAnotherNoteAmp;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class AutoAmpCompleteVisV2 extends SequentialCommandGroup {

        public AutoAmpCompleteVisV2(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        AmpAutoCommands ampac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        LimelightVision llv,
                        boolean innerNoteFirst) {

                addCommands( // note
                                ampac.setAmpStart(swerve, transfer, intake, cf),

                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 20)),

                                cf.transferNoteToShooterCommand(),

                                ampac.pickupCenter1_2(cf, pf, swerve, transfer, intake, innerNoteFirst),

                                // if note in intake go shoot it or try the adjacent one if not
                                Commands.either(
                                                ampac.moveShootCenter1_2(cf, pf, swerve, innerNoteFirst),
                                                new GetAnotherNoteAmp(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake()),

                                ampac.pickUpNoteAfterShootVision(pf, cf, swerve, transfer, intake,
                                                innerNoteFirst),

                                Commands.either(
                                                ampac.moveShootCenter1_2(cf, pf, swerve, !innerNoteFirst),
                                                new GetAnotherNoteAmp(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake()));

        }

}