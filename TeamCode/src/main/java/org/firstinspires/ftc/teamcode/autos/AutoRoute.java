package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.paths.BlueAutoPaths;
import org.firstinspires.ftc.teamcode.paths.RedAutoPaths;

/** Paths MainAuto use. */
public interface AutoRoute {

    // One method per path, same name as visualizer.
    Path garden();

    Path gardenShoot();

    Path flower1();

    Path flower1Shoot();

    Path flower2Prep();

    Path flower2();

    Path pushPrep();

    Path flower2Shoot();

    Path parkPrep();

    Path park();

    // Adapters. Inherit export methods. Renamed path = compile error here.
    final class Red extends RedAutoPaths implements AutoRoute {
    }

    final class Blue extends BlueAutoPaths implements AutoRoute {
    }
}
