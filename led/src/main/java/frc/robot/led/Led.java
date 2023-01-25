package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface Led {
    void start(AddressableLEDBuffer buffer, int length);
    void run(AddressableLEDBuffer buffer, int length);
    void end(AddressableLEDBuffer buffer, int length);
}