package frc.robot.util;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;


public class ControlMap{

    
    public static CommandXboxController driver_controls = new CommandXboxController(Constants.DriverConstants.driverPort);
    public static Joystick gunner_controls = new Joystick(Constants.DriverConstants.gunnerPort);
    public static Joystick gunner_controls_2 = new Joystick(Constants.DriverConstants.gunnerPort2);
    public static JoystickButton gunnerButton1 = new JoystickButton(gunner_controls, 1);
    public static JoystickButton gunnerButton2 = new JoystickButton(gunner_controls, 2);
    public static JoystickButton gunnerButton3 = new JoystickButton(gunner_controls, 3);
    public static JoystickButton gunnerButton4 = new JoystickButton(gunner_controls, 4);
    public static JoystickButton gunnerButton5 = new JoystickButton(gunner_controls, 5);
    public static JoystickButton gunnerButton6 = new JoystickButton(gunner_controls, 6);
    public static JoystickButton gunnerButton7 = new JoystickButton(gunner_controls, 7);
    public static JoystickButton gunnerButton8 = new JoystickButton(gunner_controls, 8);
    public static JoystickButton gunnerButton9 = new JoystickButton(gunner_controls, 9);
    public static JoystickButton gunnerButton10 = new JoystickButton(gunner_controls, 10);
    public static JoystickButton gunnerButton11 = new JoystickButton(gunner_controls, 11);
    public static JoystickButton gunnerButton12 = new JoystickButton(gunner_controls, 12);
    public static JoystickButton gunnerButton13 = new JoystickButton(gunner_controls_2, 13);
    public static JoystickButton gunnerButton14 = new JoystickButton(gunner_controls_2, 14);

}