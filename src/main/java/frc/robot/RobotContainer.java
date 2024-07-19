// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Arm.JogArm;
import frc.robot.commands.Autos.Autos.SourceAmpAutoCommands;
import frc.robot.commands.Autos.SubwfrStart.SubwooferAutoCommands;
import frc.robot.commands.Climber.PositionClimber;
import frc.robot.commands.Climber.PrepositionForClimb;
import frc.robot.commands.Drive.AlignTargetOdometry;
import frc.robot.commands.Drive.AlignToNote;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.commands.Test.MovePickupShootTest;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.ShootingData;
import frc.robot.utils.ViewArmShooterByDistance;
import monologue.Annotations.Log;
import monologue.Logged;

public class RobotContainer implements Logged {
        /* Subsystems */

        final IntakeSubsystem m_intake = new IntakeSubsystem();

        final TransferSubsystem m_transfer = new TransferSubsystem();

        final ArmSubsystem m_arm = new ArmSubsystem();

        final ClimberSubsystem m_climber = new ClimberSubsystem();

        final PowerDistribution m_pd = new PowerDistribution(1, ModuleType.kRev);

        public final LimelightVision m_llv = new LimelightVision();

        public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

        public final SendableChooser<String> m_batteryChooser = new SendableChooser<String>();

        private final CommandXboxController driver = new CommandXboxController(0);

        private final CommandXboxController codriver = new CommandXboxController(1);

        private final CommandXboxController setup = new CommandXboxController(2);

        final ShooterSubsystem m_shooter = new ShooterSubsystem();

        SendableChooser<Command> autoChooser;

        ShootingData m_sd = new ShootingData();

        public final PathFactory m_pf;// = new PathFactory(m_swerve);

        public final CommandFactory m_cf;

        public final AutoFactory m_af;

        private SubwooferAutoCommands m_sac;

        private SourceAmpAutoCommands m_srcac;

        BooleanSupplier keepAngle;

        public BooleanSupplier fieldRelative;

        private boolean forceRobotRelative = false;

        // private Trigger logShotTrigger;

        EventLoop checkAutoSelectLoop;

        private BooleanEvent doAutoSetup;

        private Trigger canivoreCheck;
        private Trigger lobshootTrigger;
        public CANBusStatus canInfo;
        @Log.NT(key = "canivoreutil")
        public float busUtil;
        final SwerveSubsystem m_swerve = new SwerveSubsystem();

        Command testCommand() {
                // Load the path you want to follow using its name in the GUI
                PathPlannerPath path = PathPlannerPath.fromPathFile("SourceToCenter4");

                // Create a path following command using AutoBuilder. This will also trigger
                // event markers.
                return AutoBuilder.followPath(path);
        }

        public RobotContainer() {

                m_pf = new PathFactory(m_swerve);

                m_cf = new CommandFactory(m_swerve, m_shooter, m_arm, m_intake, m_transfer, m_sd);

                // registerNamedCommands();

                m_sac = new SubwooferAutoCommands(m_swerve, m_intake, m_shooter, m_arm, m_transfer, m_cf, m_pf, m_sd);
                m_srcac = new SourceAmpAutoCommands(m_swerve, m_intake, m_transfer, m_cf, m_pf, m_sd);

                m_af = new AutoFactory(m_pf, m_cf, m_sac, m_srcac, m_swerve, m_shooter, m_arm, m_intake,
                                m_transfer, m_sd);

                if (RobotBase.isReal()) {
                        Pref.deleteUnused();
                        Pref.addMissing();
                }

                autoChooser = AutoBuilder.buildAutoChooser();

                m_pd.resetTotalEnergy();

                SmartDashboard.putData("TestCan", this.testAllCan().ignoringDisable(true));

                SmartDashboard.putData("ClearStickyFaults", this.clearAllStickyFaultsCommand().ignoringDisable(true));

                SmartDashboard.putData("PD", m_pd);

                SmartDashboard.putData("ViewArmShooterData",
                                new ViewArmShooterByDistance(m_cf, m_sd, m_arm).ignoringDisable(true));
                // SmartDashboard.putData("RotateToNote",
                // new RotateToFindNote(m_swerve, 45));
                // SmartDashboard.putData("AlignToNote",
                // new AutoAlignNote(m_swerve, 1, false));

                SmartDashboard.putData("PP 5metersX",
                                m_cf.autopathfind(new Pose2d(), 0, 0));

                SmartDashboard.putData("RunTestPickupAndShoot",
                                new MovePickupShootTest(m_cf, m_swerve, m_arm, m_transfer, m_intake,
                                                m_shooter, m_sd,
                                                CameraConstants.rearCamera.camname,
                                                4));

                // SmartDashboard.putData("TrapTuneToPref",
                // new TrapTune(m_swerve));

                SmartDashboard.putData("Set Robot At 0",
                                Commands.runOnce(() -> m_swerve.resetPoseEstimator(new Pose2d())));

                SmartDashboard.putData("SetAngleKp",
                                Commands.runOnce(() -> m_swerve.setTurnKp()));

                SmartDashboard.putData("S2NX4", Commands.sequence(
                                Commands.runOnce(() -> m_swerve.targetNote = 4),
                                m_cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),
                                Commands.waitSeconds(.5),
                                new RunPPath(m_swerve, m_pf.getPath(
                                                sbwfrpaths.TEST3.name()))));

                configureDriverBindings();

                configureCodriverBindings();

                configureChoosers();

                m_intake.setPID();

                configureSetupBindings();

                // m_shooter.setTopKpKdKi();

                configureCommandScheduler();

                setDefaultCommands();

                checkAutoSelectLoop = new EventLoop();

                doAutoSetup = new BooleanEvent(checkAutoSelectLoop, m_af::checkChoiceChange);

                doAutoSetup.ifHigh(() -> setAutoData());

                if (RobotBase.isReal()) {

                        canInfo = CANBus.getStatus("CV1");
                        busUtil = canInfo.BusUtilization;

                        canivoreCheck = new Trigger(
                                        () -> !canInfo.Status.isOK() || canInfo.Status.isError()
                                                        || canInfo.Status.isWarning());
                        canivoreCheck.onTrue(Commands.runOnce(() -> logCanivore()));

                        lobshootTrigger = new Trigger(
                                        () -> m_transfer.noteAtIntake() && m_swerve.aligning
                                                        && m_swerve.alignedToTarget && m_arm.getAtSetpoint()
                                                        && m_shooter.bothAtSpeed() && driver.leftBumper().getAsBoolean()
                                                        && m_swerve.getDistanceFromStageEdge() < 4.2
                                                        && m_swerve.getDistanceFromStageEdge() > 3
                                                        && Math.abs(m_swerve
                                                                        .getFieldRelativeSpeeds().vxMetersPerSecond) < 1
                                                        && m_swerve.getY() < 5);

                        lobshootTrigger.onTrue(m_cf.transferNoteToShooterCommand());
                }

                // ela portForwardCameras();
        }

        public void logCanivore() {
                log("errcanivore", canInfo.Status.isError());
                log("warncanivore", canInfo.Status.isWarning());
                log("okcanivore", canInfo.Status.isOK());
                log("canivoredesc", canInfo.Status.getDescription());
        }

        private void configureDriverBindings() {

                // KEEP IN BUTTON ORDER

                fieldRelative = () -> !forceRobotRelative;

                keepAngle = () -> false;
                // align for speaker shots

                driver.leftTrigger().whileTrue(
                                Commands.parallel(
                                                new AlignTargetOdometry(
                                                                m_swerve,
                                                                () -> -driver.getLeftY(),
                                                                () -> driver.getLeftX(),
                                                                () -> driver.getRightX(), false),
                                                m_cf.positionArmRunShooterByDistance(false)));

                driver.rightBumper().and(driver.a().negate()).onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> forceRobotRelative = true),
                                                m_intake.startIntakeCommand(),
                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians),
                                                Commands.parallel(
                                                                new TransferIntakeToSensor(m_transfer, m_intake,
                                                                                m_swerve, 20),
                                                                m_cf.rumbleCommand(driver))))
                                .onFalse(Commands.runOnce(() -> forceRobotRelative = false));

                // pick up notes with vision align
                driver.rightBumper().and(driver.a()).onTrue(
                                Commands.sequence(Commands.runOnce(() -> forceRobotRelative = true),
                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians),
                                                Commands.waitUntil(() -> m_arm.getAtSetpoint()),
                                                m_intake.startIntakeCommand(),
                                                Commands.deadline(
                                                                new TransferIntakeToSensor(m_transfer,
                                                                                m_intake, m_swerve, 120),
                                                                new AlignToNote(
                                                                                m_swerve,
                                                                                CameraConstants.rearCamera.camname,
                                                                                () -> -driver.getLeftY(),
                                                                                () -> driver.getLeftX(),
                                                                                () -> driver.getRightX()),
                                                                m_cf.rumbleCommand(driver))))
                                .onFalse(Commands.runOnce(() -> forceRobotRelative = false));

                // align with amp corner for lob shots
                driver.leftBumper().whileTrue(
                                Commands.parallel(
                                                new AlignTargetOdometry(
                                                                m_swerve,
                                                                () -> -driver.getLeftY(),
                                                                () -> driver.getLeftX(),
                                                                () -> driver.getRightX(), true),
                                                m_cf.rumbleCommand(driver),
                                                m_cf.positionArmRunShooterByDistanceLobShot(false)))

                                .onFalse(
                                                Commands.parallel(
                                                                m_shooter.stopShooterCommand(),
                                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians)));

                // shoot
                driver.rightTrigger().onTrue(
                                Commands.sequence(
                                                Commands.waitUntil(() -> m_arm.getAtSetpoint()),
                                                m_cf.transferNoteToShooterCommand(),
                                                m_shooter.stopShooterCommand(),
                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians),
                                                m_intake.stopIntakeCommand()));

                driver.b().onTrue(m_shooter.stopShooterCommand());

                driver.x().onTrue(m_shooter.startShooterCommand(3500, 5));

                driver.povUp().onTrue(Commands.parallel(new PositionClimber(m_climber, 10, .6),
                                new PrepositionForClimb(m_swerve, m_cf, true),
                                Commands.runOnce(() -> forceRobotRelative = true)));

                driver.povDown().onTrue(Commands.parallel(new PositionClimber(m_climber, 10, .6),
                                new PrepositionForClimb(m_swerve, m_cf, false),
                                Commands.runOnce(() -> forceRobotRelative = true)));

                // driver.povUp().onTrue(m_shooter.increaseRPMCommand(100));

                // driver.povDown().onTrue(m_shooter.decreaseRPMCommand(100));

                // driver.povRight().onTrue(Commands.runOnce(() ->
                // m_arm.incrementArmAngle(10)));

                // driver.povLeft().onTrue(Commands.runOnce(() -> m_arm.decrementArmAngle(10)));

                driver.start().onTrue(Commands.runOnce(() -> m_swerve.zeroGyro()));

                driver.rightStick().or(driver.leftStick()).onTrue(Commands.runOnce(() -> forceRobotRelative = false));

                driver.back().onTrue(
                                Commands.sequence(
                                                m_cf.positionArmRunShooterSpecialCase(45,
                                                                4750),
                                                Commands.waitSeconds(2),
                                                m_cf.transferNoteToShooterCommand(),
                                                new WaitCommand(1))
                                                .andThen(
                                                                Commands.parallel(
                                                                                m_arm.setGoalCommand(
                                                                                                ArmConstants.pickupAngleRadians),
                                                                                m_shooter.stopShooterCommand())));

        }

        private void configureCodriverBindings() {
                // CoDriver
                // left and right triggers are for climber
                codriver.leftTrigger().and(codriver.povUp().negate()).and(codriver.a().negate())
                                .whileTrue(m_climber.raiseClimberArmsCommand(0.3))
                                .onFalse(m_climber.stopClimberCommand());

                codriver.leftTrigger().and(codriver.povUp()).and(codriver.a().negate())
                                .whileTrue(m_climber.raiseClimberArmsCommand(0.6))
                                .onFalse(m_climber.stopClimberCommand());

                codriver.leftTrigger().and(codriver.a()).onTrue(new PositionClimber(m_climber, 10, .6));

                codriver.leftTrigger().and(codriver.b()).onTrue(new PositionClimber(m_climber, 20, .6));

                codriver.rightTrigger().and(codriver.povDown().negate())
                                .whileTrue(m_climber.lowerClimberArmsCommand(0.3))
                                .onFalse(m_climber.stopClimberCommand());

                codriver.rightTrigger().and(codriver.povDown())
                                .whileTrue(m_climber.lowerClimberArmsCommand(0.6))
                                .onFalse(m_climber.stopClimberCommand());

                // left bumper for commands used during match

                codriver.leftBumper().and(codriver.y())
                                .onTrue(m_cf.positionArmRunShooterSpecialCase(
                                                Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed));

                codriver.leftBumper().and(codriver.a())
                                .onTrue(m_arm.setGoalCommand(ArmConstants.armAngleOnBottomStopBar));

                // codriver.leftBumper().and(codriver.b()).onTrue(

                codriver.leftBumper().and(codriver.povUp())
                                .onTrue(m_sd.incArmOffsetDegreesCommand(.5));

                codriver.leftBumper().and(codriver.povDown()).onTrue(
                                m_sd.incArmOffsetDegreesCommand(-.5));

                codriver.leftBumper().and(codriver.povLeft()).onTrue(
                                m_sd.resetArmOffsetDegreesCommand());

                codriver.leftBumper().and(codriver.povRight()).onTrue(
                                m_sd.setArmOffsetDegreesCommand(2));

                // right bumper occasional use

                if (codriver.rightBumper().and(codriver.start()).getAsBoolean() && DriverStation.isDisabled())
                        Commands.runOnce(() -> m_arm.armMotor.setIdleMode(IdleMode.kCoast)).ignoringDisable(true);

                codriver.rightBumper().and(codriver.y()).whileTrue(
                                Commands.run(() -> m_swerve.wheelsAlign(), m_swerve));

                codriver.rightBumper().and(codriver.x()).onTrue(
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.absoluteResetFrontModuleEncoders()),
                                                Commands.runOnce(() -> m_swerve.absoluteResetBackModuleEncoders())));

                codriver.rightBumper().and(codriver.back()).whileTrue(
                                m_intake.reverseIntakeCommand())
                                .onFalse(m_intake.stopIntakeCommand());

        }

        private void configureSetupBindings() {
                // Setup
                // KEEP IN BUTTON ORDER
                // jogs are in case note gets stuck

                setup.a().onTrue(m_climber.lockClimberCommand());

                // setup.leftBumper().whileTrue(m_swerve.quasistaticForward());

                // setup.leftTrigger().whileTrue(m_swerve.dynamicForward());

                // setup.rightBumper().whileTrue(m_swerve.quasistaticBackward());

                // setup.rightTrigger().whileTrue(m_swerve.dynamicBackward());

                // setup.a().whileTrue(new WheelRadiusCharacterization(m_swerve));

                setup.leftBumper().whileTrue(new JogArm(m_arm, setup));

                // setup.leftBumper().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(25)));

                setup.povDown().onTrue(
                                new PositionClimber(m_climber, 5, .6));

                setup.povUp().onTrue(new PositionClimber(m_climber, 10, .6));

                setup.leftTrigger().onTrue(new PositionClimber(m_climber, 12, .6));

                setup.rightBumper().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(35)));

                setup.rightTrigger().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(40)));

                setup.x().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(45)));

                setup.y().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(50)));

                setup.povLeft().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(55)));

                setup.povRight().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(60)));

                //

                // setup.start().onTrue(new RotateToAngle(m_swerve, -90));

                // setup.back()
        }

        private void setDefaultCommands() {
                m_swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                m_swerve,
                                                () -> -driver.getLeftY(),
                                                () -> -driver.getLeftX(),
                                                () -> -driver.getRawAxis(4),
                                                fieldRelative,
                                                keepAngle));
        }

        private void configureCommandScheduler() {
                SmartDashboard.putData("CommSchd", CommandScheduler.getInstance());
        }

        // private void registerNamedCommands() {

        // }

        private void configureChoosers() {

                m_startDelayChooser.setDefaultOption("0 sec", 0.);
                m_startDelayChooser.addOption("1 sec", 1.);
                m_startDelayChooser.addOption("2 sec", 2.);
                m_startDelayChooser.addOption("3 sec", 3.);
                m_startDelayChooser.addOption("4 sec", 4.);
                m_startDelayChooser.addOption("5 sec", 5.);

                m_batteryChooser.setDefaultOption("A", "A");
                m_batteryChooser.addOption("B", "B");
                m_batteryChooser.addOption("C", "C");
                m_batteryChooser.addOption("D", "D");
                m_batteryChooser.addOption("E", "E");
                m_batteryChooser.addOption("F", "F");

                SmartDashboard.putData("DelayChooser", m_startDelayChooser);
                SmartDashboard.putData("PPAutoChooser", autoChooser);
                SmartDashboard.putData("BatteryChooser", m_batteryChooser);
        }

        public Command getPPAutCommand() {
                return autoChooser.getSelected();
        }

        void setAutoData() {
                m_af.validStartChoice = m_af.selectAndLoadPathFiles();
                if (m_af.validStartChoice >= m_af.minsbwfrauto && m_af.validStartChoice <= m_af.maxsbwfrauto) {
                }
                if (m_af.validStartChoice >= m_af.minsourceauto && m_af.validStartChoice <= m_af.maxsourceauto) {
                        m_cf.setStartPosebyAlliance(FieldConstants.sourceStartPose).runsWhenDisabled();
                }
                if (m_af.validStartChoice >= m_af.minampauto && m_af.validStartChoice <= m_af.maxampauto) {
                        m_cf.setStartPosebyAlliance(FieldConstants.ampStartPose).runsWhenDisabled();
                }
        }

        public Command testAllCan() {
                return Commands.sequence(
                                m_arm.testCan(),
                                m_climber.testCan(),
                                m_intake.testCan(),
                                m_transfer.testCan(),
                                m_shooter.testCan(),
                                m_swerve.testAllCan());
        }

        @Log.NT(key = "canOK")
        public boolean isCanOK() {
                return m_arm.armMotorConnected
                                && m_intake.intakeMotorConnected
                                && m_transfer.transferMotorConnected
                                && m_shooter.topMotorConnected && m_shooter.bottomMotorConnected
                                && m_climber.leftMotorConnected && m_climber.rightMotorConnected
                                && m_swerve.mod0connected && m_swerve.mod1connected
                                && m_swerve.mod2connected && m_swerve.mod3connected;

        }

        public Command clearAllStickyFaultsCommand() {
                return Commands.sequence(
                                m_arm.clearStickyFaultsCommand(),
                                m_climber.clearStickyFaultsCommand(),
                                m_intake.clearStickyFaultsCommand(),
                                m_transfer.clearStickyFaultsCommand(),
                                m_shooter.clearStickyFaultsCommand(),
                                m_swerve.clearStickFaultsCommand());
        }

        @Log.NT(key = "stickyfault")
        public boolean getStickyFaults() {
                return m_arm.getStickyFaults() != 0
                                && m_intake.getStickyFaults() != 0
                                && m_transfer.getStickyFaults() != 0
                                && m_shooter.getTopStickyFaults() != 0 && m_shooter.getBottomStickyFaults() != 0
                                && m_climber.getLeftStickyFaults() != 0 && m_climber.getRightStickyFaults() != 0
                                && m_swerve.getModuleStickyFaults() != 0;

        }

        public void portForwardCameras() {
                for (int port = 5800; port <= 5809; port++) {
                        PortForwarder.add(port, "limelight.frleft.local", port);
                }

        }

}