package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Centralizes PhotonCore configuration so OpModes can stay focused on control logic.
 * Enables PhotonCore and configures bulk caching for all REV hubs.
 */
public class PhotonCoreManager {
    private final List<LynxModule> hubs;

    public PhotonCoreManager(@NonNull HardwareMap hardwareMap) {
        PhotonCore.enable();
        PhotonCore.experimental.setMaximumParallelCommands(8);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Clear the manual bulk cache on all hubs to ensure the latest sensor data is available.
     */
    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
