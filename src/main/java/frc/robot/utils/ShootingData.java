// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import monologue.Annotations.Log;
import monologue.Logged;

/** Add your docs here. */
public class ShootingData implements Logged {

    public ArrayList<ShotInfo> si = new ArrayList<ShotInfo>();

    public InterpolatingDoubleTreeMap armToleranceMap = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();
    /** Shooter look up table key: feet, values: rpm */
    public InterpolatingDoubleTreeMap shotTimeMap = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap shooterRPMToleranceMap = new InterpolatingDoubleTreeMap();

    public static InterpolatingDoubleTreeMap armLobAngleMap = new InterpolatingDoubleTreeMap();
    static {

        armLobAngleMap.put(3., Units.degreesToRadians(45.0));
        armLobAngleMap.put(4.5, Units.degreesToRadians(45.0));

    }
    public static InterpolatingDoubleTreeMap shooterLobRPMMap = new InterpolatingDoubleTreeMap();
    static {

        shooterLobRPMMap.put(10., 3700.);
        shooterLobRPMMap.put(11., 3800.);
        shooterLobRPMMap.put(12., 3900.);
    }

    @Log.NT(key = "armoffsetdegrees")
    private double armOffsetDegrees = 0;
    private double maxarmoffsetdegrees = ArmConstants.maxShootOffsetDegrees;;
    private double minarmoffsetdegrees = ArmConstants.minShootOffsetDegrees;

    public ShootingData() {
        {
            si.clear();
            // distance feet, angle degrees, speed rpm, time ms, tolerance deg
            // returns meters, radians, rpm, seconds, radians
            //

            // si.add(new ShotInfo(4.00, 55, 3100, 300, 2, 10));
            si.add(new ShotInfo(4.25, 58, 3000, 300, 2, 10));
            si.add(new ShotInfo(5.25, 52, 3000, 300, 2, 10));
            si.add(new ShotInfo(6.25, 47, 3000, 300, 2, 10));
            si.add(new ShotInfo(7.25, 43, 3000, 300, 2, 10));
            si.add(new ShotInfo(8.25, 40, 3000, 300, 2, 10));
            si.add(new ShotInfo(9.25, 37, 3250, 300, 2, 10));
            si.add(new ShotInfo(10.25, 35, 3500, 300, 2, 10));
            si.add(new ShotInfo(11.25, 32.5, 3500, 300, 2, 10));
            si.add(new ShotInfo(12.25, 30.5, 3500, 300, 2, 10));
            si.add(new ShotInfo(13.25, 28.5, 3500, 300, 2, 10));
            si.add(new ShotInfo(14.25, 27.5, 3750, 300, 2, 10));
            si.add(new ShotInfo(15.25, 26.5, 4000, 300, 2, 10));
            si.add(new ShotInfo(16.25, 25.5, 4000, 300, 2, 10));
            si.add(new ShotInfo(17.25, 24, 4250, 300, 2, 10));
            si.add(new ShotInfo(18.25, 23.5, 4500, 300, 2, 10));
            si.add(new ShotInfo(19.25, 22, 4600, 300, 2, 10));

        }

        /** Arm angle look up table key: meters, values: degrees */

        for (int i = 0; i < si.size(); i++) {
            armAngleMap.put(si.get(i).getDistanceMeters(),
                    si.get(i).getArmRads());
        }
        for (int i = 0; i < si.size(); i++) {
            armToleranceMap.put(si.get(i).getDistanceMeters(),
                    si.get(i).getToleranceRads());
        }

        for (int i = 0; i < si.size(); i++) {
            shooterRPMMap.put(si.get(i).getDistanceMeters(), si.get(i).getSpeedRPM());
        }

        for (int i = 0; i < si.size(); i++) {
            shooterRPMToleranceMap.put(si.get(i).getDistanceMeters(), si.get(i).getToleranceRPMPCT());
        }
        for (int i = 0; i < si.size(); i++) {
            shotTimeMap.put(si.get(i).getDistanceMeters(), si.get(i).getTimeSec());
        }
    }

    private void setArmOffsetDegrees(double val) {
        armOffsetDegrees = Math.min(Math.max(val, minarmoffsetdegrees), maxarmoffsetdegrees);
    }

    public Command setArmOffsetDegreesCommand(double val) {
        return Commands.runOnce(() -> setArmOffsetDegrees(val));
    }

    private void resetArmOffsetDegrees() {
        armOffsetDegrees = 0;
    }

    public Command resetArmOffsetDegreesCommand() {
        return Commands.runOnce(() -> resetArmOffsetDegrees());
    }

    public double getArmOffsetDegrees() {
        return armOffsetDegrees;
    }

    public double getArmOffsetRadians() {
        return Units.degreesToRadians(armOffsetDegrees);
    }

    private void incArmOffsetDegrees(double increment) {
        double temp = armOffsetDegrees + increment;
        armOffsetDegrees = Math.min(Math.max(temp, minarmoffsetdegrees), maxarmoffsetdegrees);
    }

    public Command incArmOffsetDegreesCommand(double val) {
        return Commands.runOnce(() -> incArmOffsetDegrees(val));
    }

    public class ShotInfo {
        private final double distanceFeet;
        private final double speedRPM;
        private final double armDegrees;
        private final double timeMs;
        private final double toleranceDegrees;
        private final double toleranceRPMPCT;

        /**
         * Constructs a new ShotInfo.
         * 
         * @param distance  of shot in feet
         * @param speed     The speed of the shot, in RPM.
         * @param arm       The angle of the arm, in degrees.
         * @param timems    Time from shooter to speaker
         * @param tolerance Arm tolerance in degrees
         */
        public ShotInfo(double distanceFeet, double armDegrees, double speedRPM, double timeMs,
                double toleranceDegrees, double toleranceRPMPCT) {
            this.distanceFeet = distanceFeet;
            this.armDegrees = armDegrees;
            this.speedRPM = speedRPM;
            this.timeMs = timeMs;
            this.toleranceDegrees = toleranceDegrees;
            this.toleranceRPMPCT = toleranceRPMPCT;
        }

        public double getDistanceMeters() {
            return Units.feetToMeters(distanceFeet);
        }

        public double getArmRads() {
            return Units.degreesToRadians(armDegrees);
        }

        public double getSpeedRPM() {
            return this.speedRPM;
        }

        public double getToleranceRads() {
            return Units.degreesToRadians(toleranceDegrees);
        }

        public double getToleranceRPMPCT() {
            return this.toleranceRPMPCT;
        }

        public double getTimeSec() {
            return timeMs / 1000;
        }

    }
}
