// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FieldCentricDifferentialDrive;
import frc.robot.subsystems.Drive;

public class RobotContainer {

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private Drive m_drive;
  private SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    configureSubsystems();
    configureBindings();
    configureAutos();
  }

  private void configureAutos() {
    var replanningConfig = new ReplanningConfig(false, false);
    AutoBuilder.configureLTV(m_drive::getPose2d, m_drive::resetPose, m_drive::getChassisSpeeds, m_drive::drive, 0.02,
        replanningConfig, () -> false, m_drive);
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autoChooser);
  }

  private void configureSubsystems() {
    m_drive = new Drive(1, 2, 3, 4);
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(m_drive.driveCommand(() -> MathUtil.applyDeadband(-m_controller.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(-m_controller.getRightX(), 0.1)));

    m_controller.a().whileTrue(new FieldCentricDifferentialDrive(
        () -> -MathUtil.applyDeadband(m_controller.getLeftY(), 0.1),
        () -> -MathUtil.applyDeadband(m_controller.getLeftX(), 0.1),
        m_drive));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
