package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueAutoPaths;
import org.firstinspires.ftc.teamcode.paths.RedAutoPaths;

/** Paths MainAuto use. */
public interface AutoRoute {

    // One method per path, same name as visualizer.
    Path path1();

    Path path2();

    Path path3();

    Path path4();

    Path path5();

    Path path6();

    Path path7();

    Path path8();

    // Adapters. Inherit export methods. Renamed path = compile error here.
    final class Red extends RedAutoPaths implements AutoRoute {
    }

    final class Blue extends BlueAutoPaths implements AutoRoute {
    }
}
