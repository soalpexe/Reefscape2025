// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Vision.Camera;

public class Robot extends TimedRobot {
        CommandScheduler scheduler;

        XboxController controller;
        ButtonBoard board;
        Container container;

        Timer timer;
        StructPublisher<Pose2d> publisher;

        Command routine;

        Pose2d robotPose = new Pose2d();
        int side = 0;

        public Robot() {
                scheduler = CommandScheduler.getInstance();

                controller = new XboxController(Constants.controllerID);
                board = new ButtonBoard(Constants.boardID);
                container = new Container();

                timer = new Timer();
                publisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

                routine = AutoRoutines.left3Coral(container);
        }

        @Override
        public void robotPeriodic() {
                scheduler.run();

                // container.updateRobotPose(container.getVision().getEstimates());

                // if (!container.getQuest().initialized()) container.getQuest().resetPose(robotPose);
                // container.updateRobotPose(container.getQuest().getPose());

                robotPose = container.getDrivetrain().getRobotPose();
                side = Utilities.tagToSide(container.getVision().getTagID(Camera.Front));
                
                container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
                container.updateLEDs();

                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.updateValues();

                publisher.set(robotPose);

                if (timer.hasElapsed(5)) {
                        System.gc();
                        timer.reset();
                }
        }

        @Override
        public void autonomousInit() {
                scheduler.cancelAll();
                routine.schedule();
        }

        @Override
        public void teleopInit() {
                scheduler.cancelAll();
        }

        @Override
        public void teleopPeriodic() {
                container.driveJoysticks(
                        controller.getLeftX(),
                        controller.getLeftY(),
                        controller.getRightX(),
                        controller.getLeftTriggerAxis() > 0.1
                ).schedule();

                if (side != -1) {
                        if (controller.getPOV() == 90) container.driveToPose(Constants.leftTargets[side]).schedule();
                        if (controller.getPOV() == 270) container.driveToPose(Constants.rightTargets[side]).schedule();
                }

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
}
