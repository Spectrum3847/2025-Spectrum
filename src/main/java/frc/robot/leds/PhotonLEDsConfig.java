package frc.robot.leds;

import frc.robot.leds.LedFull.LedFullConfig;
import lombok.*;

public class PhotonLEDsConfig extends LedFullConfig {
    @Getter private final int port = 8;

    public PhotonLEDsConfig() {
        super();
        setPort(port);
    }
}
