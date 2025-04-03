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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        
        Arm arm;
        Elevator elevator;
        Vision vision;
        Climber climber;

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

                arm = new Arm(Constants.Arm.pivotID, Constants.Arm.rollersID, Constants.Arm.coralRangeID, Constants.Arm.algaeRangeID);
                elevator = new Elevator(Constants.Elevator.leftID, Constants.Elevator.rightID, Constants.canivoreID);
                vision = new Vision(Constants.Vision.leftID, Constants.Vision.rightID);
                climber = new Climber(Constants.Climber.winchID, Constants.canivoreID);

                lights = new CANdle(Constants.lightsID);

                mode = Mode.Coral;
                coralLevel = Elevator.Position.High_Stow;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public Drivetrain getDrivetrain() {
                return drivetrain;
        }

        public Arm getArm() {
                return arm;
        }

        public Elevator getElevator() {
                return elevator;
        }

        public Vision getVision() {
                return vision;
        }

        public Mode getMode() {
                return mode;
        }

        public Elevator.Position getCoralLevel() {
                return coralLevel;
        }

        public Elevator.Position getAlgaeLevel() {
                return algaeLevel;
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

        public Command modeCoral() {
                return new Command() {
                        public void initialize() {
                                mode = Mode.Coral;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command modeAlgae() {
                return new Command() {
                        public void initialize() {
                                mode = Mode.Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command targetLow() {
                return new Command() {
                        public void initialize() {
                                coralLevel = Elevator.Position.High_Stow;
                                algaeLevel = Elevator.Position.Low_Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command targetMedium() {
                return new Command() {
                        public void initialize() {
                                coralLevel = Elevator.Position.L3_Coral;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command targetHigh() {
                return new Command() {
                        public void initialize() {
                                coralLevel = Elevator.Position.L4_Coral;
                                algaeLevel = Elevator.Position.High_Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command driveJoysticks(double leftX, double leftY, double rightX, boolean slowed) {
                ChassisSpeeds speeds = new ChassisSpeeds(
                        -leftY * Constants.Drivetrain.maxSpeed,
                        -leftX * Constants.Drivetrain.maxSpeed,
                        -rightX * Constants.Drivetrain.maxAngularSpeed
                );
                Command command = drivetrain.driveSpeeds(speeds, slowed)
                
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(drivetrain);

                return command;
        }

        public Command stow() {
                Command command = Commands.either(
                        Commands.sequence(
                                arm.setPosition(Arm.Position.Low_Stow),
                                elevator.setPosition(Elevator.Position.High_Stow)
                        ),
                        Commands.sequence(
                                arm.setPosition(Arm.Position.High_Stow),
                                elevator.setPosition(Elevator.Position.High_Stow)
                        ),

                        () -> arm.hasAlgae()
                )
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runIntake() {
                Command command = Commands.sequence(
                        arm.setPosition(Arm.Position.High_Stow),
                        Commands.either(
                                Commands.sequence(
                                        elevator.setPosition(Elevator.Position.Low_Stow),
                                        Commands.parallel(
                                                arm.setPosition(Arm.Position.Intake_Coral),
                                                arm.intakeCoral()
                                        ),
                                        arm.setPosition(Arm.Position.High_Stow)
                                ),
                                Commands.parallel(
                                        arm.setPosition(Arm.Position.Intake_Algae),
                                        elevator.setPosition(algaeLevel),
                                        arm.intakeAlgae()
                                ),

                                () -> mode == Mode.Coral
                        )
                )
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runTeleOuttake() {
                Command command = Commands.either(
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

                                        () -> Utilities.inTolerance(Elevator.Position.Low_Stow.value - elevator.getPosition(), 0.4)
                                ),

                                () -> Utilities.inTolerance(Elevator.Position.High_Algae.value - elevator.getPosition(), 0.4)
                        ),

                        () -> mode == Mode.Coral
                )
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runAutoOuttake() {
                Command command = Commands.either(
                        Commands.sequence(
                                arm.setPosition(Arm.Position.High_Stow),
                                elevator.setPosition(coralLevel),
                                arm.setPosition(coralLevel == Elevator.Position.L4_Coral ? Arm.Position.L4_Coral : Arm.Position.High_Stow),
                                Commands.waitSeconds(0.3),
                                arm.outtakeCoral()
                        ),
                        Commands.sequence(
                                arm.setPosition(Arm.Position.Low_Stow),
                                elevator.setPosition(Elevator.Position.High_Stow),
                                Commands.waitSeconds(0.3),
                                arm.outtakeAlgae(-0.6)
                        ),

                        () -> mode == Mode.Coral
                )
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command climb() {
                return Commands.either(
                        climber.setPosition(Climber.Position.Deploy),
                        climber.setPosition(Climber.Position.Stow),

                        () -> Utilities.inTolerance(Climber.Position.Stow.value - climber.getPosition(), 0.2)
                );
        }
}
