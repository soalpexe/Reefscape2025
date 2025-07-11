// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;

public class Robot extends TimedRobot {
        CommandScheduler scheduler;
        Timer timer;

        XboxController controller;
        ButtonBoard board;
        Container container;
        
        Pose2d robotPose = new Pose2d();
        StructPublisher<Pose2d> publisher;

        Command routine;
        Button[] binds = new Button[] {
                Button.kLeftBumper,
                Button.kRightBumper,

                Button.kA,
                Button.kY
        }; 

        public Robot() {
                scheduler = CommandScheduler.getInstance();
                timer = new Timer();

                controller = new XboxController(Constants.controllerID);
                board = new ButtonBoard(Constants.boardID);
                container = new Container();

                publisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

                NamedCommands.registerCommand("Intake", container.intake());
                NamedCommands.registerCommand("Outtake", container.autoOuttake());
                NamedCommands.registerCommand("Stow", container.stow());

                routine = AutoBuilder.buildAuto("2-1Coral");
        }

        @Override
        public void robotPeriodic() {
                scheduler.run();

                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.updateValues();

                container.drivetrain.updateRobotHeight(container.elevator.getPosition());
                container.updateLEDs();

                robotPose = container.drivetrain.getRobotPose();
                publisher.set(robotPose);
        }

        @Override
        public void autonomousInit() {
                scheduler.cancelAll();

                container.drivetrain.seedFieldCentric();
                routine.schedule();
        }

        @Override
        public void teleopInit() {
                scheduler.cancelAll();
        }

        @Override
        public void teleopPeriodic() {
                if (board.getButtonPressed(Action.Reset)) scheduler.cancelAll();

                container.driveJoysticks(
                        controller.getLeftX(),
                        controller.getLeftY(),
                        controller.getRightX(),
                        controller.getLeftTriggerAxis() > 0.1
                ).schedule();
                
                if (controller.getLeftBumperButtonPressed()) container.scheduleOnly(container.intake());
                if (controller.getRightBumperButtonPressed()) container.scheduleOnly(container.teleOuttake());

                if (controller.getAButtonPressed()) container.scheduleOnly(container.stow());
                if (controller.getYButtonPressed()) container.climb().schedule();

                if (controller.getXButtonPressed()) container.drivetrain.seedFieldCentric();

                if (board.getButtonPressed(Action.Mode_Coral)) container.mode = Container.Mode.Coral;
                if (board.getButtonPressed(Action.Mode_Algae)) container.mode = Container.Mode.Algae;
                
                if (board.getButtonPressed(Action.Target_Low)) container.targetLow();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh();
        }
}
