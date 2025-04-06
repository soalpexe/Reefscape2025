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

                routine = AutoRoutines.center1Coral(container);
        }

        @Override
        public void robotPeriodic() {
                scheduler.run();

                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.updateValues();

                if (timer.hasElapsed(5)) {
                        System.gc();
                        timer.reset();
                }

                container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
                container.updateLEDs();

                robotPose = container.getDrivetrain().getRobotPose();
                publisher.set(robotPose);
        }

        @Override
        public void autonomousInit() {
                scheduler.cancelAll();

                container.getDrivetrain().seedFieldCentric();
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
                
                if (controller.getLeftBumperButtonPressed()) {
                        if (container.getMode() == Container.Mode.Coral) container.intakeCoral().schedule();
                        if (container.getMode() == Container.Mode.Algae) container.intakeAlgae().schedule();
                }
                if (controller.getRightBumperButtonPressed()) container.teleOuttake().schedule();

                if (controller.getAButtonPressed()) container.stow().schedule();
                if (controller.getYButtonPressed()) container.climb().schedule();

                if (controller.getXButtonPressed()) container.getDrivetrain().seedFieldCentric();

                if (board.getButtonPressed(Action.Mode_Coral)) container.modeCoral().schedule();
                if (board.getButtonPressed(Action.Mode_Algae)) container.modeAlgae().schedule();
                
                if (board.getButtonPressed(Action.Target_Low)) container.targetLow().schedule();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium().schedule();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh().schedule();

        }
}
