// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Utilities;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
        static Rotation2d redPerspective = Rotation2d.k180deg, bluePerspective = Rotation2d.kZero;
        boolean appliedPerspective = false;

        SwerveRequest.FieldCentric fieldCentric;
        AutoFactory autoConfigs;

        double antiTipping;

        public Drivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {
                super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfigs, modules);

                fieldCentric = new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.Drivetrain.maxSpeed * 0.1).withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                autoConfigs = new AutoFactory(
                        this::getRobotPose,
                        this::resetPose,
                        this::followTrajectory,
                        true,
                        this
                );
        }

        public Pose2d getRobotPose() {
                Pose2d estimate = getState().Pose;
                
                return new Pose2d(
                        estimate.getX(),
                        estimate.getY(),
                        getRotation3d().toRotation2d()
                );
        }
        
        public void updateRobotHeight(double height) {
                antiTipping = (30 - height) / 30;
        }

        public void setControl(ChassisSpeeds speeds, boolean slowed) {
                double translationSlow = slowed ? 0.15 : antiTipping;
                double headingSlow = slowed ? 0.5 : antiTipping;

                setControl(fieldCentric
                        .withVelocityX(speeds.vxMetersPerSecond * translationSlow)
                        .withVelocityY(speeds.vyMetersPerSecond * translationSlow)
                        .withRotationalRate(speeds.omegaRadiansPerSecond * headingSlow)
                );
        }

        public Command driveSpeeds(ChassisSpeeds speeds, boolean slowed) {
                return run(() -> setControl(speeds, slowed));
        }

        public Command driveSpeeds(ChassisSpeeds speeds) {
                return run(() -> setControl(speeds, false));
        }

        public void followTrajectory(SwerveSample sample) {
                Pose2d robotPose = getRobotPose();

                ChassisSpeeds speeds = new ChassisSpeeds(
                        sample.vx + Constants.Drivetrain.translationPID.calculate(robotPose.getX(), sample.x),
                        sample.vy + Constants.Drivetrain.translationPID.calculate(robotPose.getY(), sample.y),
                        sample.omega + Constants.Drivetrain.translationPID.calculate(Utilities.getRadians(robotPose), sample.heading)
                );

                setControl(speeds, false);
        }

        @Override
        public void periodic() {
                if (!appliedPerspective || DriverStation.isDisabled()) {
                        DriverStation.getAlliance().ifPresent(allianceColor -> {
                                setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red ? redPerspective : bluePerspective
                                );

                                appliedPerspective = true;
                        });
                }
        }
}
