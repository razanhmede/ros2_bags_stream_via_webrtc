set(MV OFF)
set(PIP_INSTALL_OPTIONS "")

if("${PIP_INSTALL_PREFIX}" STREQUAL "--user")
    set(PIP_INSTALL_OPTIONS "--user")
elseif(NOT "${PIP_INSTALL_PREFIX}" STREQUAL "")
    set(PIP_INSTALL_OPTIONS "--prefix=${PIP_INSTALL_PREFIX}")
    if(NOT "" STREQUAL "")
        set(MV ON)
    endif()
endif()

execute_process(
    COMMAND /usr/bin/python3.10 -m pip install --upgrade --force-reinstall --no-deps --no-cache-dir --no-index ${PIP_INSTALL_OPTIONS} --find-links=/home/razanh/Inmind/WebRTC/ros2_bags_stream_via_webrtc/build/signaling-server/opentera_webrtc_signaling_server/dist opentera_webrtc.signaling_server
    WORKING_DIRECTORY /home/razanh/Inmind/WebRTC/ros2_bags_stream_via_webrtc/build/signaling-server/opentera_webrtc_signaling_server
)

if(${MV} STREQUAL "ON")
    execute_process(
        # The format is the same between --user (where pip should be installed) and --prefix (where the package is installed)
        COMMAND bash -c "rsync -a --remove-source-files ${PIP_INSTALL_PREFIX}/lib/python3.8/site-packages/ ${PIP_INSTALL_PREFIX}/lib// 2> /dev/null"
        WORKING_DIRECTORY /home/razanh/Inmind/WebRTC/ros2_bags_stream_via_webrtc/build/signaling-server/opentera_webrtc_signaling_server
    )
    execute_process(
        COMMAND find ${PIP_INSTALL_PREFIX}/lib -depth -type d -empty -delete
        WORKING_DIRECTORY /home/razanh/Inmind/WebRTC/ros2_bags_stream_via_webrtc/build/signaling-server/opentera_webrtc_signaling_server
    )
endif()

execute_process(
    COMMAND /usr/bin/cmake -E touch /home/razanh/Inmind/WebRTC/ros2_bags_stream_via_webrtc/build/signaling-server/cmake/pip_utils/install.stamp
    WORKING_DIRECTORY /home/razanh/Inmind/WebRTC/ros2_bags_stream_via_webrtc/build/signaling-server/opentera_webrtc_signaling_server
)
