// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        
        Arm arm;
        Elevator elevator;
        Climber climber;

        Vision vision;
        CANdle lights;

        Mode mode;
        Elevator.Position coralLevel, algaeLevel;

        public enum Mode {
                Coral,
                Algae
        }

        public Container() {
                drivetrain = new Drivetrain(
                        Constants.Drivetrain.drivetrainConfigs,

                        Constants.Drivetrain.frontLeftConfigs,
                        Constants.Drivetrain.frontRightConfigs,
                        Constants.Drivetrain.backLeftConfigs,
                        Constants.Drivetrain.backRightConfigs
                );

                arm = new Arm(Constants.Arm.pivotID, Constants.Arm.rollersID, Constants.Arm.coralRangeID, Constants.Arm.algaeRangeID, Constants.canivoreID);
                elevator = new Elevator(Constants.Elevator.leftID, Constants.Elevator.rightID);
                climber = new Climber(Constants.Climber.winchID);
                
                vision = new Vision(Constants.Vision.frontID);
                lights = new CANdle(Constants.lightsID);

                mode = Mode.Coral;
                coralLevel = Elevator.Position.L4_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public void updateRobotPose(Pose2d estimate) {
                if (Utilities.isValidPose(estimate)) {
                        Pose2d pose = new Pose2d(
                                estimate.getX(),
                                estimate.getY(),
                                drivetrain.getPigeon2().getRotation2d()
                        );

                        drivetrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()));
                }
        }

        public void updateRobotPose(Pose2d[] estimates) {
                for(Pose2d estimate : estimates) {
                       updateRobotPose(estimate);
                }
        }

        public void updateLEDs() {
                if (mode == Mode.Coral) lights.setLEDs(255, 0, 255);
                else lights.setLEDs(0, 255, 0);
        }

        public void targetLow() {
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public void targetMedium() {
                coralLevel = Elevator.Position.L3_Coral;
        }

        public void targetHigh() {
                coralLevel = Elevator.Position.L4_Coral;
                algaeLevel = Elevator.Position.High_Algae;
        }

        public void scheduleOnly(Command command) {
                CommandScheduler.getInstance().cancelAll();
                command.schedule();
        }
        
        public Command driveJoysticks(double leftX, double leftY, double rightX, boolean slowed) {
                ChassisSpeeds speeds = new ChassisSpeeds(
                        leftY * Constants.Drivetrain.maxSpeed,
                        leftX * Constants.Drivetrain.maxSpeed,
                        -rightX * Constants.Drivetrain.maxAngularSpeed
                );
                
                return drivetrain.driveSpeeds(speeds, slowed);
        }

        public Command stow() {
                return Commands.either(
                        Commands.sequence(
                                arm.setPosition(Arm.Position.Low_Stow),
                                elevator.setPosition(Elevator.Position.High_Stow)
                        ),
                        Commands.sequence(
                                arm.setPosition(Arm.Position.High_Stow),
                                elevator.setPosition(Elevator.Position.Low_Stow)
                        ),

                        () -> arm.hasAlgae()
                );
        }

        public Command intake() {
                return Commands.either(
                        Commands.sequence(
                                arm.setPosition(Arm.Position.High_Stow),
                                elevator.setPosition(Elevator.Position.Low_Stow),
                                Commands.parallel(
                                        arm.setPosition(Arm.Position.Intake_Coral),
                                        arm.intakeCoral()
                                ),
                                arm.setPosition(Arm.Position.High_Stow)
                        ),
                        Commands.sequence(
                                arm.setPosition(Arm.Position.High_Stow),
                                Commands.parallel(
                                        arm.setPosition(Arm.Position.Intake_Algae),
                                        elevator.setPosition(algaeLevel),
                                        arm.intakeAlgae()
                                )
                        ),

                        () -> mode == Mode.Coral
                );
        }

        public Command teleOuttake() {
                return Commands.either(
                        Commands.either(
                                arm.outtakeCoral(),
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.High_Stow),
                                        elevator.setPosition(coralLevel),
                                        arm.setPosition(coralLevel == Elevator.Position.L4_Coral ? Arm.Position.L4_Coral : Arm.Position.High_Stow)
                                ),
                                
                                () -> Utilities.inTolerance(coralLevel.value - elevator.getPosition(), 0.4)
                        ),
                        Commands.either(
                                Commands.parallel(
                                        elevator.setPosition(Elevator.Position.End_Barge),
                                        arm.setPosition(Arm.Position.High_Stow),
                                        Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                arm.outtakeAlgae(-1)
                                        )
                                ),
                                Commands.either(
                                        arm.outtakeAlgae(-0.6),
                                        Commands.sequence(
                                                elevator.setPosition(Elevator.Position.High_Algae),
                                                arm.setPosition(Arm.Position.Start_Barge)
                                        ),

                                        () -> Utilities.inTolerance(Elevator.Position.High_Stow.value - elevator.getPosition(), 0.4)
                                ),

                                () -> Utilities.inTolerance(Elevator.Position.High_Algae.value - elevator.getPosition(), 0.4)
                        ),

                        () -> mode == Mode.Coral
                );
        }

        public Command autoOuttake() {
                return Commands.either(
                        Commands.sequence(
                                arm.setPosition(Arm.Position.High_Stow),
                                elevator.setPosition(coralLevel),
                                arm.setPosition(coralLevel == Elevator.Position.L4_Coral ? Arm.Position.L4_Coral : Arm.Position.High_Stow),
                                Commands.waitSeconds(0.2),
                                arm.outtakeCoral()
                        ),
                        Commands.sequence(
                                arm.setPosition(Arm.Position.Low_Stow),
                                elevator.setPosition(Elevator.Position.High_Stow),
                                Commands.waitSeconds(0.2),
                                arm.outtakeAlgae(-0.6)
                        ),

                        () -> mode == Mode.Coral
                );
        }

        public Command climb() {
                return Commands.either(
                        climber.setPosition(Climber.Position.Deploy),
                        climber.setPosition(Climber.Position.Stow),

                        () -> Utilities.inTolerance(Climber.Position.Stow.value - climber.getPosition(), 0.2)
                );
        }
}
