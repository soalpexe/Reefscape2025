// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
                        container.getDrivetrain().startTrajectory("1-CL"),

                        container.getDrivetrain().followTrajectory("1-CL"),
                        container.runAutoOuttake(),
                        container.getDrivetrain().followTrajectory("CL-SL"),
                        container.runIntake(),
                        container.getDrivetrain().followTrajectory("SL-BL"),
                        container.runAutoOuttake(),
                        container.getDrivetrain().followTrajectory("BL-SL"),
                        container.runIntake(),
                        container.getDrivetrain().followTrajectory("SL-BR"),
                        container.runAutoOuttake()
                );
        }
}
