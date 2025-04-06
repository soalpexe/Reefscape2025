// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;

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

        public static boolean bindPressed(XboxController controller, Button[] buttons) {
                for (Button button : buttons) {
                        boolean pressed = controller.getRawButtonPressed(button.value);
                        if (pressed) return true;
                }

                return false;
        }
}
