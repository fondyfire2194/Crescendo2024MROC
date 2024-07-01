// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.LimelightHelpers;

public class TakeLLSnapshot extends Command {
  /** Creates a new TakeLLSnapshot. */
  private final String m_camtable;
  private int lpctr;

  public TakeLLSnapshot(String camtable) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_camtable = camtable;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lpctr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("SNAPSHOT", lpctr++);
  //  "\"ROM\""
    NetworkTableInstance.getDefault().getTable("limelight-frleft");// .getEntry("pipeline").setNumber(1);

    String p = NetworkTableInstance.getDefault().getTable("limelight-frleft").toString();

    SmartDashboard.putString("LLp", p);

    LimelightHelpers.takeSnapshot(LimelightHelpers.getLimelightNTTable("limelight-frleft").getPath(), "Test");
 LimelightHelpers.takeSnapshot("limelight-frleft", "Test");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
