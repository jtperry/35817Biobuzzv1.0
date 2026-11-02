package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueAuto1TipFarPaths;
import org.firstinspires.ftc.teamcode.paths.RedAuto1TipFarPaths;

/** Paths MainAuto1TipFar use. */
public interface AutoRoute1TipFar {

    // One method per path, same name as visualizer.
    Path path1();

    // Adapters. Inherit export methods. Renamed path = compile error here.
    final class Red extends RedAuto1TipFarPaths implements AutoRoute1TipFar {
    }

    final class Blue extends BlueAuto1TipFarPaths implements AutoRoute1TipFar {
    }
}
