package raidzero.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidzero.robot.subsystems.Swerve;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
	private double MaxAngularRate = 1.5 * Math.PI;

	private final CommandXboxController joystick = new CommandXboxController(0);
	private final Swerve drivetrain = Swerve.system();

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * Constants.Swerve.STICK_DEADBAND)
			.withRotationalDeadband(MaxAngularRate * Constants.Swerve.STICK_DEADBAND)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	private final Telemetry logger = new Telemetry(MaxSpeed);

	public RobotContainer() {
		configureBindings();

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}

		drivetrain.registerTelemetry(logger::telemeterize);

		SmartDashboard.putData(drivetrain.getField2d());
	}

	private void configureBindings() {
		drivetrain.setDefaultCommand(
				drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
						.withVelocityY(-joystick.getLeftX() * MaxSpeed)
						.withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

		// * CONTROLS SUBJECT TO CHANGE

		// Brake on A button press
		joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

		// reset the pigeon2 heading on right bumper press
		joystick.rightBumper().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0)));

		// reset the field-centric heading on left bumper press
		joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}