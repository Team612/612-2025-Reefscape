package frc.robot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //constants
  private int constantXboxControllerPortNumber = 0;

  // instantiate things here
  private XboxController controller = new XboxController(constantXboxControllerPortNumber);

  // object that interacts with the I2C port to which the sensor is connected to
  private I2C sensor = new I2C(I2C.Port.kOnboard,0x17);

  // Read a 16-bit signed value from the sensor
  private int read16Bit(int register) {
    byte[] buffer = new byte[2];
    sensor.read(register, 2, buffer);
    // return (buffer[1] << 8) | (buffer[0]);
    return ((buffer[1] & 0xFF) << 8) | (buffer[0] & 0xFF); 
  }

  // the low byte for the heading returns garbage data for some reason so we are only reading the high byte
  public int getHeading(){
    byte[] buffer = new byte[2];
    sensor.read(0x25,2,buffer);
    return ((buffer[0] & 0xFF));
  }

  // returns the binary data from a register in 1s and 0s
  public String debug(int register){
    byte[] buffer = new byte[2];
    sensor.read(register, 2, buffer);

    String binaryStringIndex0 = String.format("%8s", Integer.toBinaryString(buffer[0] & 0xFF)).replace(' ', '0');
    String binaryStringIndex1 = String.format("%8s", Integer.toBinaryString(buffer[1] & 0xFF)).replace(' ', '0');
    
    return (binaryStringIndex0 + " " + binaryStringIndex1);
  }

  // Get X and Y movement deltas
  public int getDeltaX() {
    return read16Bit(0x20);
  }
  public int getDeltaY() {
    return read16Bit(0x22);
  }

  public void reset(){
    sensor.write(0x07, 0x01);
    try {
      // three second delay to attempt to reduce errors
      // reduce or remove this if you are tired of waiting
      Thread.sleep(3000);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      System.out.println("Error Pausing");
    }
  }

  public void setLinearScalar(double scalar) {
    // Check if the scalar is out of bounds
    if (scalar < 0.872 || scalar > 1.127)
      return;

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    byte rawScalar = (byte) ((scalar - 1.0) * 1000 + 0.5);

    sensor.write(0x04, rawScalar);
  }

  public void setAngularScalar(double scalar) {
    // Check if the scalar is out of bounds
    if (scalar < 0.872 || scalar > 1.127)
        return;

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    byte rawScalar = (byte) ((scalar - 1.0) * 1000 + 0.5);

    sensor.write(0x05, rawScalar);
  }

  public boolean calibrateImu(int numSamples, boolean waitUntilDone) {
    // Check if the number of samples is out of bounds
    if (numSamples < 1 || numSamples > 255)
        return false;

    // Write the number of samples to the device
    sensor.write(0x06, numSamples);

    // Wait 1 sample period (2.4ms) to ensure the register updates
    try {
        Thread.sleep(3);
    } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return false;
    }

    // Do we need to wait until the calibration finishes?
    if (!waitUntilDone)
      return true;

    // Wait for the calibration to finish, which is indicated by the IMU
    // calibration register reading zero, or until we reach the maximum number
    // of read attempts
    for (int numAttempts = numSamples; numAttempts > 0; numAttempts--) {
      // Read the gryo calibration register value
      byte[] buffer = new byte[2];
      sensor.read(0x06, 2, buffer);

      byte calibrationValue = buffer[0];
      // byte calibrationValue = buffer[1];

      // Check if calibration is done
      if (calibrationValue == 0)
          return true;

      // Give a short delay between reads. As of firmware v1.0, samples take
      // 2.4ms each, so 3ms should guarantee the next sample is done. This
      // also ensures the max attempts is not exceeded in normal operation
      try {
          Thread.sleep(3);
      } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
      }
    }

    // Max number of attempts reached, calibration failed
    return false;
  }

  public Robot() {}

  @Override
  public void robotPeriodic() {
    int deltaX = getDeltaX();
    int deltaY = getDeltaY();
    int heading = getHeading();
    double actualX = deltaX * 0.0003;
    double actualY = deltaY * 0.0003;
    double actualDegrees = heading * 360 / 256;

    SmartDashboard.putNumber("X", actualX);
    SmartDashboard.putNumber("Y", actualY);
    SmartDashboard.putNumber("Heading", actualDegrees);

    SmartDashboard.putString("register0x20", debug(0x20));
    SmartDashboard.putString("register0x21", debug(0x21));
    SmartDashboard.putString("register0x22", debug(0x22));
    SmartDashboard.putString("register0x23", debug(0x23));
    SmartDashboard.putString("register0x24", debug(0x24));
    SmartDashboard.putString("register0x25", debug(0x25));
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (controller.getYButtonPressed()){
      reset();
    }
    if (controller.getBButtonPressed()){
      calibrateImu(254,true);
    }
    if (controller.getAButtonPressed()){
      setLinearScalar(1.05);
    }
    if (controller.getXButtonPressed()){
      setAngularScalar(0.9897222);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}