package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Drivetrain;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kControls;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    //Robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain(); //Drivetrain subsystem

    //Driver's controller
    private static final CommandXboxController primary = new CommandXboxController(kControls.PRIMARY_PORT);

    // Speed factor to reduce drive speed (e.g., 50% speed)
    private static final double slowFactor = 0.5;

    /*
     * Constructor for the robot container. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Configure button bindings
        configureButtonBindings();

        // Configure default commands
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Get controller inputs
                    double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(), kControls.DRIVE_DEADBAND);
                    double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(), kControls.DRIVE_DEADBAND);
                    double rot = -MathUtil.applyDeadband(primary.getRightX(), kControls.DRIVE_DEADBAND);

                    // Apply slow factor if B button is pressed
                    if (primary.b().getAsBoolean()) {
                        ySpeed *= slowFactor;
                        xSpeed *= slowFactor;
                        rot *= slowFactor;
                    }

                    drivetrain.drive(ySpeed, xSpeed, rot, true);
                },
                drivetrain));
        }

    /*
     * Defining button -> command mappings. Buttons can
     * created by instantiating a Trigger class.
     */
    private void configureButtonBindings() {
        //Example of how to bind a button to a command using command-based framework:
        //primary.a().whileTrue(new ExampleCommand());
    }
    /*
     * Passes autonomous command to main Robot class.
     */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}


