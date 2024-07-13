package frc.robot.utils;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

public class SparkMaxUtil {
  public static final int disableFramePeriod = 65535;
  private static final PeriodicFrame[] periodicFrames = PeriodicFrame.values();

  public static void configureFollower(CANSparkMax follower) {
    for (PeriodicFrame frame : periodicFrames) {
      follower.setPeriodicFramePeriod(frame, disableFramePeriod);
    }
  }
}
