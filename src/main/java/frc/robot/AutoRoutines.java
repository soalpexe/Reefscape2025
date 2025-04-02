// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Vision.Camera;

public class AutoRoutines {
        public static Command leave(Container container) {
                return Commands.sequence(
                        container.getDrivetrain().driveSpeeds(new ChassisSpeeds(-1, 0, 0)),
                        Commands.waitSeconds(3),
                        container.getDrivetrain().driveSpeeds(new ChassisSpeeds())
                );
        }

        public static Command left3Coral(Container container) {
                return Commands.sequence(
                        container.targetHigh(),

                        container.getDrivetrain().followTrajectory("1-C"),
                        container.getDrivetrain().alignTag(
                                container.getVision().getOffsetX(Camera.Right),
                                container.getVision().getOffsetY(Camera.Right),
                                container.getVision().getTagID(Camera.Right)
                        ),
                        container.runAutoOuttake(),
                        
                        Commands.parallel(
                                container.getDrivetrain().followTrajectory("CL-SL"),
                                Commands.sequence(
                                        Commands.waitSeconds(0.2),
                                        container.stow()
                                )
                        ),
                        container.runIntake(),

                        container.getDrivetrain().followTrajectory("SL-B"),
                        container.getDrivetrain().alignTag(
                                container.getVision().getOffsetX(Camera.Right),
                                container.getVision().getOffsetY(Camera.Right),
                                container.getVision().getTagID(Camera.Right)
                        ),
                        container.runAutoOuttake(),

                        Commands.parallel(
                                container.getDrivetrain().followTrajectory("BL-SL"),
                                Commands.sequence(
                                        Commands.waitSeconds(0.2),
                                        container.stow()
                                )
                        ),
                        container.runIntake(),

                        container.getDrivetrain().followTrajectory("SL-B"),
                        container.getDrivetrain().alignTag(
                                container.getVision().getOffsetX(Camera.Left),
                                container.getVision().getOffsetY(Camera.Left),
                                container.getVision().getTagID(Camera.Left)
                        ),
                        container.runAutoOuttake()
                );
        }
}
