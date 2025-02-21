package frc.robot.util;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


public class ControlMap{
    private static int DRIVER_PORT = Constants.DriverConstants.driverPort;
    private static int GUNNER_PORT = Constants.DriverConstants.gunnerPort;
    private static int DRIVER_PORT_BUTTONS = 2;
    private static int GUNNER_PORT_BUTTONS = 3;
    
    public static CommandXboxController driver_joystick = new CommandXboxController(DRIVER_PORT);
    public static CommandXboxController gunner_joystick = new CommandXboxController(GUNNER_PORT);
    public static Joystick driver_buttons = new Joystick(DRIVER_PORT_BUTTONS);
    public static Joystick gunner_buttons = new Joystick(GUNNER_PORT_BUTTONS);
}