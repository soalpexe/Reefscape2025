// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
        public static Command leave(Container container) {
                return Commands.race(
                        container.getDrivetrain().driveSpeeds(new ChassisSpeeds(0, -1, 0)),
                        Commands.waitSeconds(0.8)
                );
        }
}
