// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;


public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;

  SwerveDrive m_swerveDrive;

  public RobotContainer() {
    this.m_frontLeft = new SwerveModule((Constants.Swerve.Motors.kFrontLeftVars));
    this.m_frontRight = new SwerveModule((Constants.Swerve.Motors.kFrontRightVars));
    this.m_backLeft = new SwerveModule((Constants.Swerve.Motors.kBackLeftVars));
    this.m_backRight = new SwerveModule((Constants.Swerve.Motors.kBackRightVars));
    
    this.m_swerveDrive = new SwerveDrive(this.m_frontLeft, this.m_frontRight, this.m_backLeft, this.m_backRight);

    configureBindings();
  }

  
  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(
      new DriveSwerve(
        m_swerveDrive,
        () -> m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.rightBumper().getAsBoolean()
      )
    );

    m_driverController.rightTrigger(0.1).whileTrue(new DriveSwerve(
      m_swerveDrive,
      () -> m_driverController.getRightTriggerAxis(),
      () -> 0.0,
      () -> 0.0,
      () -> m_driverController.rightBumper().getAsBoolean()
    ));
  }

  
  public Command getAutonomousCommand() {
    return null;
  }
}
