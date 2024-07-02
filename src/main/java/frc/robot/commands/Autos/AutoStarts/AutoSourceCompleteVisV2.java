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
import frc.robot.commands.Autos.Autos.GetAnotherNoteSource;
import frc.robot.commands.Autos.Autos.SourceAutoCommands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class AutoSourceCompleteVisV2 extends SequentialCommandGroup {

        public AutoSourceCompleteVisV2(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SourceAutoCommands srcac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        LimelightVision llv,
                        double switchoverdistance,
                        boolean innerNoteFirst) {

                addCommands( // note
                                srcac.setSourceStart(swerve, transfer, intake, cf),
                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 20)),
                                cf.transferNoteToShooterCommand(),

                                srcac.pickupCenter4_5(cf, pf, swerve, transfer, intake, innerNoteFirst),
                                // if note in intake go shoot it or try to find one
                                Commands.either(
                                                srcac.moveShootCenter4_5(cf, pf, swerve, innerNoteFirst),
                                                new GetAnotherNoteSource(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake()),

                                srcac.pickUpNoteAfterShootVision(pf, cf, swerve, transfer, intake,
                                                innerNoteFirst),

                                Commands.either(
                                                srcac.moveShootCenter4_5(cf, pf, swerve, !innerNoteFirst),
                                                new GetAnotherNoteSource(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake()));
        }

}