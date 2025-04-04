// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities {
        public static Alliance getAlliance() {
                return DriverStation.getAlliance().get();
        }

        public static double getRadians(Pose2d pose) {
                return pose.getRotation().getRadians();
        }

        public static boolean inTolerance(double error, double tolerance) {
                return Math.abs(error) < tolerance;
        }

        public static boolean isValidPose(Pose2d pose) {
                return !pose.equals(new Pose2d()) && pose != null;
        }

        public static int tagToSide(int tagID) {
                if (tagID == 7 || tagID == 18) return 0;
                if (tagID == 6 || tagID == 19) return 0;
                if (tagID == 11 || tagID == 20) return 0;
                if (tagID == 10 || tagID == 21) return 0;
                if (tagID == 9 || tagID == 22) return 0;
                if (tagID == 8 || tagID == 17) return 0;

                return -1;
        }
}
