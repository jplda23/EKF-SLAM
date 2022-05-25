#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jplda23/catkin_ws/src/mbot_simulation_sa/resources/packages/mbot_bringup"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jplda23/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jplda23/catkin_ws/install/lib/python2.7/dist-packages:/home/jplda23/catkin_ws/build/mbot_bringup/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jplda23/catkin_ws/build/mbot_bringup" \
    "/usr/bin/python2" \
    "/home/jplda23/catkin_ws/src/mbot_simulation_sa/resources/packages/mbot_bringup/setup.py" \
     \
    build --build-base "/home/jplda23/catkin_ws/build/mbot_bringup" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jplda23/catkin_ws/install" --install-scripts="/home/jplda23/catkin_ws/install/bin"
