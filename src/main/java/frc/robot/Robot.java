// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Vision.Camera;

public class Robot extends TimedRobot {
        CommandScheduler scheduler;

        XboxController controller;
        ButtonBoard board;

        Container container;

        Timer timer;
        AutoChooser autoChooser;

        double leftOffsetX, leftOffsetY;
        double rightOffsetX, rightOffsetY;

        public Robot() {
                scheduler = CommandScheduler.getInstance();

                controller = new XboxController(Constants.controllerID);
                board = new ButtonBoard(Constants.boardID);
                container = new Container();

                timer = new Timer();
                autoChooser = new AutoChooser();

                autoChooser.addCmd("Leave", () -> AutoRoutines.leave(container));
                autoChooser.addCmd("Left 3 Coral", () -> AutoRoutines.left3Coral(container));
        }

        @Override
        public void robotPeriodic() {
                scheduler.run();

                container.updateRobotPose(container.getVision().getEstimates());
                container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
                container.updateLEDs();

                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.putData(autoChooser);
                SmartDashboard.updateValues();

                if (timer.hasElapsed(5)) {
                        System.gc();
                        timer.reset();
                }
                
                leftOffsetX = container.getVision().getOffsetX(Camera.Left);
                leftOffsetX = container.getVision().getOffsetY(Camera.Left);

                rightOffsetX = container.getVision().getOffsetX(Camera.Right);
                rightOffsetX = container.getVision().getOffsetY(Camera.Right);
        }

        @Override
        public void autonomousInit() {
                scheduler.cancelAll();
                autoChooser.selectedCommand().schedule();
        }

        @Override
        public void autonomousPeriodic() {}

        @Override
        public void autonomousExit() {}

        @Override
        public void teleopInit() {
                scheduler.cancelAll();
        }

        @Override
        public void teleopPeriodic() {
                if (controller.getLeftTriggerAxis() > 0.9) container.getDrivetrain().alignTag(leftOffsetX, leftOffsetY, container.getVision().getTagID(Camera.Left)).schedule();
                if (controller.getRightTriggerAxis() > 0.9) container.getDrivetrain().alignTag(rightOffsetX, rightOffsetY, container.getVision().getTagID(Camera.Right)).schedule();

                container.driveJoysticks(
                        controller.getLeftX(),
                        controller.getLeftY(),
                        controller.getRightX(),
                        controller.getLeftTriggerAxis() > 0.1
                ).schedule();

                if (controller.getXButtonPressed()) container.getDrivetrain().seedFieldCentric();

                if (board.getButtonPressed(Action.Mode_Coral)) container.modeCoral().schedule();
                if (board.getButtonPressed(Action.Mode_Algae)) container.modeAlgae().schedule();

                if (controller.getAButtonPressed()) container.stow().schedule();
                if (controller.getYButtonPressed()) container.climb().schedule();
                
                if (board.getButtonPressed(Action.Target_Low)) container.targetLow().schedule();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium().schedule();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh().schedule();

                if (controller.getLeftBumperButtonPressed()) container.runIntake().schedule();
                if (controller.getRightBumperButtonPressed()) container.runTeleOuttake().schedule();
        }

        @Override
        public void teleopExit() {}
}
