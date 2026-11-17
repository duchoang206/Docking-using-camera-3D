"""Entry point: ros2 run camera3d_docking cloud_saver"""


def main(args=None):
    from camera3d_docking._env import ensure_clean_python

    ensure_clean_python()  # trước khi import numpy (tránh numpy 2.x trong ~/.local)
    from camera3d_docking.ros.cloud_saver import main as run

    run(args)


if __name__ == "__main__":
    main()
