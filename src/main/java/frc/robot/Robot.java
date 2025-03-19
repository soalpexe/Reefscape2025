// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision.Camera;

public class Robot extends TimedRobot {
        XboxController controller;
        ButtonBoard board;

        Container container;

        Field2d field;
        Pose2d leftTarget, rightTarget, centerTarget;

        Timer timer;
        AutoChooser autoChooser;

        public Robot() {
                controller = new XboxController(Constants.controllerID);
                board = new ButtonBoard(Constants.boardID);
                container = new Container();

                field = new Field2d();

                timer = new Timer();
                autoChooser = new AutoChooser();

                autoChooser.addCmd("Leave", () -> AutoRoutines.leave(container));
                autoChooser.addCmd("Left 3 Coral", () -> AutoRoutines.left3Coral(container));
        }

        @Override
        public void robotPeriodic() {
                CommandScheduler.getInstance().run();

                container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
                container.updateLEDs();

                container.updateRobotPose(container.getVision().getEstimate(Camera.Front));
                int side = Utilities.getClosestSide(Constants.Vision.centerPoses, container.getDrivetrain().getRobotPose());

                leftTarget = Constants.Vision.leftPoses[side];
                rightTarget = Constants.Vision.rightPoses[side];
                centerTarget = Constants.Vision.centerPoses[side];

                Pose2d robotPose = container.getDrivetrain().getRobotPose();
                field.setRobotPose(
                        robotPose.getX(),
                        robotPose.getY(),
                        robotPose.getRotation()
                );

                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.putData("Robot Pose", field);

                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
                SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

                if (container.getMode() == Container.Mode.Coral) {
                        SmartDashboard.putBoolean("Target Low", container.getCoralLevel() == Elevator.Position.L2_Coral);
                        SmartDashboard.putBoolean("Target Medium", container.getCoralLevel() == Elevator.Position.L3_Coral);
                        SmartDashboard.putBoolean("Target High", container.getCoralLevel() == Elevator.Position.L4_Coral);
                }

                else {
                        SmartDashboard.putBoolean("Target Low", container.getAlgaeLevel() == Elevator.Position.Low_Algae);
                        SmartDashboard.putBoolean("Target High", container.getAlgaeLevel() == Elevator.Position.High_Algae);

                        SmartDashboard.putBoolean("Medium", false);
                }

                SmartDashboard.putBoolean("Coral Mode", container.getMode() == Container.Mode.Coral);
                SmartDashboard.putBoolean("Algae Mode", container.getMode() == Container.Mode.Algae);

                SmartDashboard.putBoolean("Has Coral", container.getArm().hasCoral());
                SmartDashboard.putBoolean("Has Algae", container.getArm().hasAlgae());

                SmartDashboard.putData("Auto Chooser", autoChooser);

                SmartDashboard.updateValues();

                if (timer.hasElapsed(5)) {
                        System.gc();
                        timer.reset();
                }
        }

        @Override
        public void autonomousInit() {
                CommandScheduler.getInstance().cancelAll();
                
                autoChooser.selectedCommand().schedule();
        }

        @Override
        public void autonomousPeriodic() {}

        @Override
        public void autonomousExit() {}

        @Override
        public void teleopInit() {
                CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void teleopPeriodic() {
                if (board.getButton(Action.Align_Left)) container.driveToPose(leftTarget).schedule();
                if (board.getButton(Action.Align_Right)) container.driveToPose(rightTarget).schedule();
                if (board.getButton(Action.Align_Center)) container.driveToPose(centerTarget).schedule();

                container.driveJoysticks(
                        controller.getLeftX(),
                        controller.getLeftY(),
                        controller.getRightX(),
                        controller.getLeftTriggerAxis() > 0.2
                ).schedule();

                if (controller.getXButtonPressed()) container.getDrivetrain().seedFieldCentric();

                if (board.getButtonPressed(Action.Mode_Coral)) container.modeCoral().schedule();
                if (board.getButtonPressed(Action.Mode_Algae)) container.modeAlgae().schedule();

                if (controller.getAButtonPressed()) container.stow().schedule();
                
                if (board.getButtonPressed(Action.Target_Low)) container.targetLow().schedule();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium().schedule();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh().schedule();

                if (controller.getLeftBumperButtonPressed()) container.runIntake().schedule();
                if (controller.getRightBumperButtonPressed()) container.runTeleOuttake().schedule();
        }

        @Override
        public void teleopExit() {}
}
