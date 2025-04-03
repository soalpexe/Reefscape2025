// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Utilities;

public class Vision extends SubsystemBase {
        String leftID, rightID;

        public enum Camera {
                Left,
                Right
        }

        public Vision(String leftID, String rightID) {
                this.leftID = leftID;
                this.rightID = rightID;
        }

        String toString(Camera camera) {
                switch (camera) {
                        case Left: return leftID;
                        case Right: return rightID;

                        default: return null;
                }
        }

        public int getTagID(Camera camera) {
                String cameraID = toString(camera);
                return (int)LimelightHelpers.getNTEntry(cameraID, "tid").getInteger(-1);
        }

        public double getOffsetX(Camera camera) {
                String cameraID = toString(camera);
                double offset = Math.tan(Math.toRadians(LimelightHelpers.getNTEntry(cameraID, "tx").getDouble(0))) * getOffsetY(camera);

                return getTagID(camera) != -1 ? offset : 0;
        }

        public double getOffsetY(Camera camera) {
                String cameraID = toString(camera);
                double offset = 0.75 / Math.tan(Math.toRadians(LimelightHelpers.getNTEntry(cameraID, "ty").getDouble(0)));

                return getTagID(camera) != -1 ? offset : 8;
        }

        public Pose2d getEstimate(Camera camera) {
                String cameraID = toString(camera);

                Pose2d redEstimate = LimelightHelpers.getRedPoseEstimate(cameraID);
                Pose2d blueEstimate = LimelightHelpers.getBluePoseEstimate(cameraID);

                return Utilities.getAlliance() == Alliance.Red ? redEstimate : blueEstimate;
        }

        public Pose2d[] getEstimates() {
                Pose2d[] estimates = new Pose2d[] {
                        getEstimate(Camera.Left),
                        getEstimate(Camera.Right)
                };

                return estimates;
        }
        
        @Override
        public void periodic() {}
}
