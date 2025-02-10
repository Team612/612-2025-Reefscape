package frc.robot.Controls;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControlMap{
    private static int driverStationButton = 0;
    

    public static Joystick gunner_buttons = new Joystick(driverStationButton);
}