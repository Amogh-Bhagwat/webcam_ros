Code to publish webcam data to a rostopic. A subscriber displays this data on
the screen.
Additional ros service is added to save the current frame. The service
subscribes to the webcam data requires an optional filename parameter.
Usage for pubisher -

    rosrun webcam_ros webcam_pub

Usage for subscriber -

    rosrun webcam_ros webcam_sub

Usage for action server and client -

    rosrun webcam_ros blur_server
    rosrun webcam_ros blur_client _duration:=capture_duration

Usage for service -

    rosrun webcam_ros image_processing_server

For running clients -

    rosrun webcam_ros image_save_client _filename:=your_file_name _capture_fps:=capture_fps _capture_duration:=capture_duration

    rosrun webcam_ros image_crop_client _filename:=your_file_name _width:=width _height:=height
