// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.function.Supplier;

import frc.robot.util.HubStatus;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.commands.AimAtHubCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.TurnTurretCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.util.HubStatus;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Turret.TurretIOTalon;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionIOPhoton;
import frc.robot.subsystems.Vision.VisionSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.util.SendableSupplier;

public class RobotContainer {
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // // kSpeedAt12Volts desired top speed
    private double MaxSpeed = 4d;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController m_Operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final TurretSubsystem turret = new TurretSubsystem(new TurretIOTalon());
    

    private final SendableChooser<Command> autoChooser;
    private final VisionSubsystem m_Vision = new VisionSubsystem(new VisionIOPhoton("Arducam_OV9281_USB_Camera", drivetrain::addVisionMeasurement, VisionConstants.cameraToRobot1)); 
   private final VisionSubsystem m_Vision2 = new VisionSubsystem(new VisionIOPhoton("Arducam_OV9281_USB_Camera (1)", drivetrain::addVisionMeasurement,VisionConstants.cameraToRobot2)); 


    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        
        HubStatus.getFirstInactiveAlliance(); //init
        SmartDashboard.putData("Is Active Hub", new SendableSupplier<Boolean>("IsActiveHub", () -> HubStatus.isActive()));
        SmartDashboard.putData("Active Hub", new SendableSupplier<Character>("ActiveHub", () -> HubStatus.getActiveHub()));


    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        final double DPAD = 0.25;
        joystick.pov(0).onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(DPAD)));
        joystick.pov(180).onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-DPAD)));
        joystick.pov(90).onTrue(drivetrain.applyRequest(() -> drive.withVelocityY(DPAD)));
        joystick.pov(270).onTrue(drivetrain.applyRequest(() -> drive.withVelocityY(-DPAD)));

        joystick.pov(45).onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(DPAD*Math.sqrt(0.5)).withVelocityY(DPAD*Math.sqrt(0.5))));
        joystick.pov(135).onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-DPAD*Math.sqrt(0.5)).withVelocityY(DPAD*Math.sqrt(0.5))));
        joystick.pov(225).onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-DPAD*Math.sqrt(0.5)).withVelocityY(-DPAD*Math.sqrt(0.5))));
        joystick.pov(315).onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(DPAD*Math.sqrt(0.5)).withVelocityY(-DPAD*Math.sqrt(0.5))));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.a().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.a().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.b().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.b().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.x().onTrue(new AlignCommand(drivetrain));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        turret.setDefaultCommand(new TurnTurretCommand(turret, ()->m_Operator.getLeftX()));
        m_Operator.b().onTrue(new InstantCommand(()->turret.zeroEncoder()));
        //m_Operator.x().whileTrue(new InstantCommand(()->turret.pointAtAngle(Degrees.of(0))));
        //m_Operator.y().whileTrue(new InstantCommand(()->turret.pointAtAngle(Degrees.of(180))));
        m_Operator.a().whileTrue(new AimAtHubCommand(turret, ()->drivetrain.getState().Pose, ()->drivetrain.getState().Speeds));
        drivetrain.registerTelemetry(logger::telemeterize);
    }
}
