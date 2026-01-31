// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.HeadingPID;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.SetHeading;
import frc.robot.subsystems.Drive.SmoothFieldCentricFacingAngle;
import frc.robot.subsystems.Index.IndexBase;
import frc.robot.subsystems.Index.IndexIOKraken;
import frc.robot.subsystems.Index.IndexIOSim;
import frc.robot.subsystems.Intake.IntakeBase;
import frc.robot.subsystems.Intake.Extension.ExtensionIOKraken;
import frc.robot.subsystems.Intake.Extension.ExtensionIOSim;
import frc.robot.subsystems.Intake.Roller.RollerIOKraken;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Shooter.ShooterBase;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOKraken;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOKraken;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Kickup.KickupIOKraken;
import frc.robot.subsystems.Shooter.Kickup.KickupIOSim;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.Climberz.ClimberzBase;
import frc.robot.subsystems.Climberz.ClimberzIOKraken;
import frc.robot.subsystems.Climberz.ClimberzIOSim;
import frc.robot.subsystems.Climberz.ClimberzConstants;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.ShootingConstants;
import frc.robot.utility.FuelSim;
import frc.robot.utility.ShootingCalculator;
import frc.robot.utility.Telemetry;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionIOLimelight visionIO = new VisionIOLimelight();
    private final VisionBase vision = new VisionBase(visionIO, drivetrain);
    private final SetHeading autoHeading = new SetHeading(vision);
    private final ExtensionIOKraken extensionIO = new ExtensionIOKraken();
    private final RollerIOKraken rollerIOKraken = new RollerIOKraken();
    private final RollerIOSim rollerIOSim = new RollerIOSim();
    private final ExtensionIOSim extensionIOSim = new ExtensionIOSim();
    private final FlywheelIOKraken flywheelIOKraken = new FlywheelIOKraken();
    private final FlywheelIOSim flywheelIOSim = new FlywheelIOSim();
    private final HoodIOKraken hoodIOKraken = new HoodIOKraken();
    private final HoodIOSim hoodIOSim = new HoodIOSim();
    private final KickupIOKraken kickupIOKraken = new KickupIOKraken();
    private final KickupIOSim kickupIOSim = new KickupIOSim(); 
    private final IndexIOKraken indexIOKraken = new IndexIOKraken();
    private final IndexIOSim indexIOSim = new IndexIOSim();
    private final ClimberzIOKraken climberzIOKraken = new ClimberzIOKraken();
    private final ClimberzIOSim climberzIOSim = new ClimberzIOSim();
    private final IntakeBase intake;
    private final ShooterBase shooter;
    private final IndexBase index;
    private final ClimberzBase climberz;

    private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4);

    // Double tap detection for climber

    private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
    private final SmoothFieldCentricFacingAngle headingDrive = new SmoothFieldCentricFacingAngle()
    .withRedTarget(FieldPoses.redHub.getTranslation()) 
    .withBlueTarget(FieldPoses.blueHub.getTranslation())
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(0.0);
  
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        if (Robot.isSimulation()) {
        intake = new IntakeBase(extensionIOSim, rollerIOSim);
        shooter = new ShooterBase(flywheelIOSim, hoodIOSim, kickupIOSim);
        index = new IndexBase(indexIOSim);
        climberz = new ClimberzBase(climberzIOSim);
        shooter.forceHoodHomed();
        } 
        else {
        intake = new IntakeBase(extensionIO, rollerIOKraken);
        shooter = new ShooterBase(flywheelIOKraken, hoodIOKraken, kickupIOKraken);
        index = new IndexBase(indexIOKraken);
        climberz = new ClimberzBase(climberzIOKraken);
        }

        configureNamedCommands();
        configureBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser("default auto"); //pick a default
        if (Robot.isSimulation()) {
            configureFuelSim();
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

  
    private void configureBindings() {
        m_driverController.start().onTrue(Commands.runOnce(()->{
            drivetrain.seedFieldCentric();
            vision.setGyroInitialized();
        }));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

       // Dpad right + B toggles emergency mode
        Trigger emergencyModeTrigger = m_driverController.povRight().and(m_driverController.b());
        emergencyModeTrigger.onTrue(Commands.runOnce(() -> {
            autoHeading.toggleEmergencyMode();
        }));

        // Right bumper - hub heading (normal) or emergency shooting mode
        m_driverController.rightBumper().onTrue(Commands.runOnce(() -> {
            if (autoHeading.isEmergencyModeEnabled()) {
                autoHeading.toggleEmergencyShooting();
            } else {
                if (autoHeading.isEnabled()) {
                    autoHeading.disableFaceHub();
                } else {
                    autoHeading.toggleFaceHub();
                }
            }
        }));

        // Left bumper - passing heading (normal) or emergency passing mode
        m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
            if (autoHeading.isEmergencyModeEnabled()) {
                autoHeading.toggleEmergencyPassing();
            } else {
                if (autoHeading.isPassingEnabled()) {
                    autoHeading.disablePassing();
                } else {
                    autoHeading.togglePassing();
                }
            }
        }));

        // Dpad left - toggle hood retraction in emergency mode
        m_driverController.povLeft().onTrue(Commands.runOnce(() -> {
            autoHeading.toggleEmergencyHoodRetract();
        }));

        m_driverController.leftTrigger().toggleOnTrue(
            new IntakeCommand(intake)
        );

   // Climber controls - dpad down to climb (retract), dpad up to release (extend)
        m_driverController.povDown()
            .whileTrue(Commands.run(() -> climberz.climbUp(), climberz))
            .onFalse(Commands.runOnce(() -> climberz.stop(), climberz));
        
        m_driverController.povUp()
            .whileTrue(Commands.run(() -> climberz.climbDown(), climberz))
            .onFalse(Commands.runOnce(() -> climberz.stop(), climberz));
        
        // Y button - auto retract climber for 1.5 seconds
        m_driverController.y().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> climberz.climbUp(), climberz),
                Commands.waitSeconds(ClimberzConstants.homingDurationSeconds),
                Commands.runOnce(() -> climberz.stop(), climberz)
            )
        );

        m_driverController.rightTrigger(0.2).whileTrue(
                new ShootCommand(
                    shooter,
                    index,
                    () -> drivetrain.getPose(),
                    () -> drivetrain.getFieldVelocity(),
                    () -> vision.isRedAlliance(),
                    () -> autoHeading.isPassingEnabled(),
                    () -> autoHeading.isEmergencyShootingMode(),
                    () -> autoHeading.isEmergencyPassingMode(),
                    () -> autoHeading.revertFromHoodRetract()
                )
            );

        headingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        headingDrive.HeadingController.setPID(HeadingPID.headingP.get(), HeadingPID.headingI.get(), HeadingPID.headingD.get());

        drivetrain.setDefaultCommand(
            drivetrain.run(() -> {
                // Get raw joystick inputs
                double rawX = -m_driverController.getLeftY();
                double rawY = -m_driverController.getLeftX();
                double rawRot = -m_driverController.getRightX();
                
                // Apply deadband BEFORE the limiter
                rawX = Math.abs(rawX) > OperatorConstants.deadband ? rawX : 0.0;
                rawY = Math.abs(rawY) > OperatorConstants.deadband ? rawY : 0.0;
                rawRot = Math.abs(rawRot) > OperatorConstants.deadband ? rawRot : 0.0;
                
                //apply slew rate limiting
                double xSpeed = xLimiter.calculate(rawX);
                double ySpeed = yLimiter.calculate(rawY);
                double rotSpeed = rotLimiter.calculate(rawRot);
                
                // Check if driver is manually rotating
                boolean driverRotating = autoHeading.isDriverRotating(rotSpeed);

                // Determine which auto-heading mode to use
                boolean useHubHeading = autoHeading.isEnabled() && !driverRotating;
                boolean usePassingHeading = autoHeading.isPassingEnabled() && !driverRotating;

                // Update heading based on active mode
                if (useHubHeading) {
                    autoHeading.updateVirtualGoal(drivetrain.getPose(), drivetrain.getFieldVelocity());
                    headingDrive.withVirtualTarget(autoHeading.getVirtualGoal());
                } else if (usePassingHeading) {
                    autoHeading.updatePassingHeading(drivetrain.getPose());
                    // For passing, we use a fixed heading (toward wall), not a virtual target point
                    // Set a far-away point in the direction we want to face
                    boolean isRed = vision.isRedAlliance();
                    double targetX = isRed ? FieldPoses.fieldLengthMeters + 10.0 : -10.0;
                    headingDrive.withVirtualTarget(new Translation2d(targetX, drivetrain.getPose().getY()));
                } else {
                    headingDrive.clearVirtualTarget();
                }
                if (useHubHeading || usePassingHeading) {
                    drivetrain.setControl(
                        headingDrive
                        .withVelocityX(xSpeed * MaxSpeed)
                        .withVelocityY(ySpeed * MaxSpeed)
                    );
                } else {
                    drivetrain.setControl(
                        drive
                            .withVelocityX(xSpeed * MaxSpeed)
                            .withVelocityY(ySpeed * MaxSpeed)
                            .withRotationalRate(rotSpeed * MaxAngularRate)
                    );
                }
            })
        );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Shooter default command - auto-retract hood near trench OR emergency hood retract mode
        shooter.setDefaultCommand(
            shooter.run(() -> {
                // Get robot position
                Pose2d robotPose = drivetrain.getPose();
                double robotX = robotPose.getX();
                
                // Check distance to both trenches (hub X positions)
                double blueTrenchX = FieldPoses.blueHub.getX();
                double redTrenchX = FieldPoses.redHub.getX();
                
                double distanceToBlue = Math.abs(robotX - blueTrenchX);
                double distanceToRed = Math.abs(robotX - redTrenchX);
                double minDistance = Math.min(distanceToBlue, distanceToRed);
                
                // Check if emergency hood retract mode is active
                boolean emergencyRetract = autoHeading.isEmergencyHoodRetractMode();
                
                // Check if near trench (only when NOT in emergency mode)
                boolean nearTrench = !autoHeading.isEmergencyModeEnabled() && 
                    minDistance < ShootingConstants.hoodRetractionDistanceMeters;
                
                // Auto-retract if emergency retract mode OR near trench
                if (emergencyRetract || nearTrench) {
                    shooter.retractHood();
                    Logger.recordOutput("Shooter/AutoRetract", true);
                    Logger.recordOutput("Shooter/AutoRetractReason", emergencyRetract ? "EmergencyMode" : "NearTrench");
                } else {
                    Logger.recordOutput("Shooter/AutoRetract", false);
                    Logger.recordOutput("Shooter/AutoRetractReason", "None");
                }
                
                Logger.recordOutput("Shooter/DistanceToNearestTrench", minDistance);
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    

    /*
     * Final check for gyro initialization at Auto start.
     * If gyro hasn't been initialized, try one last time from cameras.
     * If cameras don't see anything, zero the gyro normally.
    */
    public void finalGyroCheck() {
        if (!vision.isGyroInitialized()) {
            boolean success = vision.forceSetYawFromCameras(drivetrain);
            if (!success) {
                // No cameras saw tags, zero normally
                drivetrain.seedFieldCentric();
                vision.setGyroInitialized();
            }
        }
    }

    public Command getIntakeHomingCommand() {
        if(!intake.isHomed()){
            return intake.homingSequence();
        }

        return null;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command getHoodHomingCommand() {
        // Assuming you have a shooter subsystem instance
        if (!shooter.isHoodHomed()) {
            return shooter.hoodHomingSequence();
        }
        return null;
    }

    private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        
        instance.registerRobot(
            RobotDimensions.robotWidthMeters,
            RobotDimensions.robotLengthMeters,
            RobotDimensions.bumperHeightMeters,
            drivetrain::getPose,
            drivetrain::getFieldVelocity
        );
        
        instance.start();
    }

    /**
     * Register all named commands for PathPlanner auto routines.
     */
    private void configureNamedCommands() {
        // ==================== SHOOTING COMMANDS ====================
        
        // "Shoot" - Full auto shooting (calculates velocity/angle, waits for spinup, feeds)
        // Runs until interrupted by PathPlanner
        NamedCommands.registerCommand("Shoot", 
            new ShootCommand(
                shooter,
                index,
                () -> drivetrain.getPose(),
                () -> drivetrain.getFieldVelocity(),
                () -> vision.isRedAlliance(),
                () -> false, // Not passing mode
                () -> false, // Not emergency shooting
                () -> false, // Not emergency passing
                () -> {}     // No callback needed for auto
            )
        );
        
        // "PrepareToShoot" - Spin up flywheel and set hood angle without feeding
        // Use this while driving to shooting position
        NamedCommands.registerCommand("PrepareToShoot",
            Commands.run(() -> {
                var solution = ShootingCalculator.calculateSolutionWithMovement(
                    drivetrain.getPose(),
                    drivetrain.getFieldVelocity(),
                    vision.isRedAlliance()
                );
                shooter.setFlywheelVelocity(solution.flywheelRPM);
                shooter.setHoodAngle(solution.hoodAngleRadians);
            }, shooter)
        );
        
        // "StopShooting" - Stop flywheel, hood, and kickup
        NamedCommands.registerCommand("StopShooting",
            Commands.runOnce(() -> {
                shooter.stopAll();
            }, shooter)
        );
        
        // ==================== PASSING COMMANDS ====================
        
        // "Pass" - Full auto passing (calculates lob trajectory, waits for spinup, feeds)
        // Runs until interrupted by PathPlanner
        NamedCommands.registerCommand("Pass",
            new ShootCommand(
                shooter,
                index,
                () -> drivetrain.getPose(),
                () -> drivetrain.getFieldVelocity(),
                () -> vision.isRedAlliance(),
                () -> true,  // Passing mode enabled
                () -> false, // Not emergency shooting
                () -> false, // Not emergency passing
                () -> {}     // No callback needed for auto
            )
        );
        
        // "PrepareToPass" - Spin up flywheel and set hood angle for passing without feeding
        NamedCommands.registerCommand("PrepareToPass",
            Commands.run(() -> {
                var solution = ShootingCalculator.calculatePassingSolution(
                    drivetrain.getPose(),
                    vision.isRedAlliance()
                );
                shooter.setFlywheelVelocity(solution.flywheelRPM);
                shooter.setHoodAngle(solution.hoodAngleRadians);
            }, shooter)
        );
        
        // ==================== INTAKE COMMANDS ====================
        
        // "StartIntake" - Deploy intake and run rollers (doesn't end on its own)
        NamedCommands.registerCommand("StartIntake",
            Commands.runOnce(() -> {
                intake.deploy();
            }, intake)
        );
        
        // "StopIntake" - Retract intake
        NamedCommands.registerCommand("StopIntake",
            Commands.runOnce(() -> {
                intake.retract();
            }, intake)
        );
        
        // "Intake" - Full intake command (deploys, runs until interrupted)
        NamedCommands.registerCommand("Intake",
            new IntakeCommand(intake)
        );
        
        // ==================== HOOD COMMANDS ====================
        
        // "RetractHood" - Set hood to minimum angle (for going under trench)
        NamedCommands.registerCommand("RetractHood",
            Commands.runOnce(() -> {
                shooter.retractHood();
            }, shooter)
        );

        // "Homing" - Run intake, hood, and climber homing in parallel
        // Add this as the FIRST command in your PathPlanner autos
        NamedCommands.registerCommand("Homing",
            Commands.parallel(
                Commands.either(
                    Commands.defer(() -> intake.homingSequence(), Set.of(intake)),
                    Commands.none(),
                    () -> !intake.isHomed()
                ),
                Commands.either(
                    Commands.defer(() -> shooter.hoodHomingSequence(), Set.of(shooter)),
                    Commands.none(),
                    () -> !shooter.isHoodHomed()
                )
            ).withTimeout(1.0)
        );
        
        // ==================== CLIMBER COMMANDS ====================
        
        // "ClimbUp" - Retract climber (lift robot) - brake mode + duty cycle
        // Runs until interrupted by PathPlanner
        NamedCommands.registerCommand("ClimbUp",
            Commands.run(() -> climberz.climbUp(), climberz)
                .finallyDo(() -> climberz.stop())
        );
        
        // "ClimbRelease" - Release climber (let springs extend) - coast mode + zero duty cycle
        // Runs until interrupted by PathPlanner
        NamedCommands.registerCommand("ClimbRelease",
            Commands.run(() -> climberz.climbDown(), climberz)
                .finallyDo(() -> climberz.stop())
        );
        
        // "ClimbStop" - Stop climber and hold position (brake mode)
        NamedCommands.registerCommand("ClimbStop",
            Commands.runOnce(() -> climberz.stop(), climberz)
        );

        NamedCommands.registerCommand("RetractClimb",
            Commands.sequence(
                    Commands.runOnce(() -> climberz.retract(), climberz),
                    Commands.waitSeconds(ClimberzConstants.homingDurationSeconds),
                    Commands.runOnce(() -> climberz.stop(), climberz)
                )
        );
    }

    
    /**
     * Check if intake needs homing
     */
    public boolean needsIntakeHoming() {
        return !intake.isHomed();
    }
    
    /**
     * Check if hood needs homing
     */
    public boolean needsHoodHoming() {
        return !shooter.isHoodHomed();
    }

}
