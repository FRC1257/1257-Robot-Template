package frc.robot.util.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.drive.CommandSnailController.DPad;

public class DriveControls {
    // Controllers
    public static final CommandSnailController driver = new CommandSnailController(0);
    public static final CommandSnailController operator = new CommandSnailController(1);

    // Useful for things that don't need to be triggered
    private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);
    private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

    // Drive controls
    public static DoubleSupplier DRIVE_FORWARD;
    public static DoubleSupplier DRIVE_STRAFE;
    public static DoubleSupplier DRIVE_ROTATE;
    public static Trigger DRIVE_SLOW;
    public static Trigger DRIVE_STOP;

    // Setup the controls
    public static void configureControls() {
        switch (Constants.driver) {
            case PROGRAMMERS:
            default:
                // Driver controls
                DRIVE_FORWARD = () -> (-driver.getLeftY());
                DRIVE_STRAFE = ()->(-driver.getLeftX());
                DRIVE_ROTATE = () -> (-driver.getRightX());
                
                // Driver Settings
                DRIVE_SLOW = driver.start();
                DRIVE_STOP = driver.x();
        }

        switch (Constants.operator) {
            case PROGRAMMERS:
            default:
                // Operator controls
                break;

                //bottom right Left joystick to intake 
        }
    }

    private static Command getRumbleCommand(CommandSnailController driver) {
        return new InstantCommand(() -> driver.rumble(1)).andThen(new WaitCommand(1)).andThen(() -> driver.rumble(0));
    }

    public static Command getRumbleBoth() {
        return getRumbleCommand(driver).alongWith(getRumbleCommand(operator));
    }

    public static Command getRumbleOperator() {
        return getRumbleCommand(operator);
    }

    public static Command getRumbleDriver() {
        return getRumbleCommand(driver);
    }
}
