// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    // Controllers
    final CommandXboxController driverXbox = new CommandXboxController(0);

    // Subsystems
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ServoSubsystem servoSubsystem = new ServoSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/Robot2025"));

    // Swerve input streams
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(1.0)
            .scaleRotation(0.5)
            .allianceRelativeControl(true)
            .robotRelative(false);

    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .robotRelative(false)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(DrivebaseConstants.DriveFastScale)
            .allianceRelativeControl(true);

    SwerveInputStream driveRobotAngularVelocitySim = driveAngularVelocitySim.copy().robotRelative(true)
            .allianceRelativeControl(false);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        drivebase.setDefaultCommand(
                RobotBase.isSimulation() ? drivebase.driveFieldOriented(driveAngularVelocitySim)
                        : drivebase.driveFieldOriented(driveAngularVelocity));

        driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
        driverXbox.back().whileTrue(drivebase.centerModulesCommand());

        driverXbox.leftTrigger().onTrue(Commands.runOnce(
                () -> driveAngularVelocity.scaleTranslation(DrivebaseConstants.DrivePrecisionScale).scaleRotation(0.3)))
                .onFalse(Commands.runOnce(
                        () -> driveAngularVelocity.scaleTranslation(DrivebaseConstants.DriveFastScale).scaleRotation(0.5)));

        driverXbox.rightTrigger().onTrue(Commands.runOnce(
                () -> driveAngularVelocity.robotRelative(true).allianceRelativeControl(false)))
                .onFalse(Commands.runOnce(
                        () -> driveAngularVelocity.robotRelative(false).allianceRelativeControl(true)));

        driverXbox.a().whileTrue(new StartEndCommand(
                () -> shooterSubsystem.runShooter(0.4),
                () -> shooterSubsystem.stopShooter(),
                shooterSubsystem));

        driverXbox.x().whileTrue(new StartEndCommand(
                () -> servoSubsystem.levelOne(),
                () -> servoSubsystem.levelOne(),
                servoSubsystem));

        driverXbox.y().whileTrue(new StartEndCommand(
                () -> servoSubsystem.LevelTwo(),
                () -> servoSubsystem.LevelTwo(),
                servoSubsystem));

        driverXbox.b().whileTrue(new StartEndCommand(
                () -> servoSubsystem.LevelThree(),
                () -> servoSubsystem.LevelThree(),
                servoSubsystem));
    }

    public void ZeroGyro() {
        drivebase.zeroGyroWithAlliance();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        configureBindings();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
