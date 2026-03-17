// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.PivotCommandAuto;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.ShootCommandAuto;
import frc.robot.Commands.ShootCommandLL;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Commands.FieldGeomUtils;

public class RobotContainer {

// Subsystems
private IntakeSubsystem intake = new IntakeSubsystem();
private PivotSubsystem pivot = new PivotSubsystem();


 private IndexerSubsystem indexer = new IndexerSubsystem();
 private ShooterSubsystem shooter = new ShooterSubsystem();
 private FeederSubsystem feeder = new FeederSubsystem();

 //Commands
 //ParallelCommandGroup shootcommandParallel = new ParallelCommandGroup(feeder.feederCommand(), indexer.indexCommand(), shooter.shootCommand());
  ParallelCommandGroup StopshootcommandParallel = new ParallelCommandGroup(feeder.stop(), indexer.stop(), shooter.stop());
 ShootCommand shootCommand= new ShootCommand(shooter, indexer, feeder);

 ShootCommandAuto shootCommandAuto = new ShootCommandAuto(shooter, indexer, feeder);
 PivotCommandAuto pivotCommandAuto = new PivotCommandAuto(pivot);


    private double MaxSpeed =  TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveAtTarget = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xbox = new CommandXboxController(0);

    private final CommandXboxController lxbox = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //FOR PRACTICE
    //private final IntakeSubsystem fuelIntake = new IntakeSubsystem();
    //private final IntakeCommand intakeCommand = new IntakeCommand(fuelIntake);
    
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        //FOR PRACTICE - putting command to pathplanner
        NamedCommands.registerCommand("ShootCommandAuto", shootCommandAuto);
        NamedCommands.registerCommand("PivotCommandAuto", pivotCommandAuto);    

        //PID

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        
        // Set PID gains for the rotation controller
    driveAtTarget.HeadingController.setPID(10.0, 0, 0.1); 
    // Enable continuous input so it doesn't spin 360 degrees unnecessarily
    driveAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-xbox.getLeftY() * 0.5 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xbox.getLeftX() * 0.5 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // xbox.povUp().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // xbox.povDown().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Reset the field-centric heading on left bumper press.
        xbox.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));



    // While holding Y, the robot points at the hub but can still be moved with the left stick
    xbox.y().whileTrue(drivetrain.applyRequest(() -> {
    var state = drivetrain.getState();
    
    // 1. Get raw inputs from xboxs
    double vx = -xbox.getLeftY() * MaxSpeed;
    double vy = -xbox.getLeftX() * MaxSpeed;

    /* 2. POSE PREDICTION (Lookahead)
     * We predict where the robot will be in 50ms to compensate for 
     * latency and the robot's own momentum.
     */
    double lookaheadSeconds = 0.050; 
    Pose2d futurePose = new Pose2d(
        state.Pose.getX() + (state.Speeds.vxMetersPerSecond * lookaheadSeconds),
        state.Pose.getY() + (state.Speeds.vyMetersPerSecond * lookaheadSeconds),
        state.Pose.getRotation()
    );

    // 3. Calculate target angles using the predicted pose
    Rotation2d targetAngle = FieldGeomUtils.getAngleToHub(futurePose);
    double distance = FieldGeomUtils.getDistanceToHub(state.Pose);

    // 4. LOGGING - Send everything to SmartDashboard for tuning
    SmartDashboard.putNumber("AutoAim/Distance Meters", distance);
    SmartDashboard.putNumber("AutoAim/Target Heading", targetAngle.getDegrees());
    SmartDashboard.putNumber("AutoAim/Current Heading", state.Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("AutoAim/Heading Error", targetAngle.minus(state.Pose.getRotation()).getDegrees());
    // This logs how much the lookahead is actually changing your target
    SmartDashboard.putNumber("AutoAim/Lookahead Offset", 
        targetAngle.minus(FieldGeomUtils.getAngleToHub(state.Pose)).getDegrees());

    // 5. Apply Request with Velocity control for smoothness
    return driveAtTarget
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withTargetDirection(targetAngle);
}));



// Intake Commands
intake.setDefaultCommand(intake.stop());
xbox.rightBumper().whileTrue(intake.intakeCommand());
xbox.rightTrigger().whileTrue(intake.outtakeCommand());

// Pivot Commands
pivot.setDefaultCommand(pivot.stopCommand());
lxbox.pov(0).whileTrue(pivot.slowUp());
lxbox.pov(180).whileTrue(pivot.slowDown());

//Test Shooter Commands
//Testing


//default shoot commands
indexer.setDefaultCommand(indexer.stop());
shooter.setDefaultCommand(shooter.stop());
feeder.setDefaultCommand(feeder.stop());


//Shoot Commands
//xbox.b().whileTrue(StopshootcommandParallel);
xbox.x().whileTrue(shootCommand);




//xbox.y().whileTrue(indexer.outtakeCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
