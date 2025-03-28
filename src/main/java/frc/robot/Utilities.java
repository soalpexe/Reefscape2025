// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

        public static Rotation2d toHeading(int tag) {
                if (tag == 8) return new Rotation2d(60);
                if (tag == 9) return new Rotation2d(120);
                if (tag == 10) return new Rotation2d(180);
                if (tag == 11) return new Rotation2d(-120);
                if (tag == 12) return new Rotation2d(-60);

                return new Rotation2d(0); 
        }
}
