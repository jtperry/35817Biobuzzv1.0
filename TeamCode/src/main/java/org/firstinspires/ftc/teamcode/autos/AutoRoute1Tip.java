package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueAuto1TipPaths;
import org.firstinspires.ftc.teamcode.paths.RedAuto1TipPaths;

/** Paths MainAuto1Tip use. */
public interface AutoRoute1Tip {

    // One method per path, same name as visualizer.
    Path path1();

    Path path2();

    // Adapters. Inherit export methods. Renamed path = compile error here.
    final class Red extends RedAuto1TipPaths implements AutoRoute1Tip {
    }

    final class Blue extends BlueAuto1TipPaths implements AutoRoute1Tip {
    }
}
