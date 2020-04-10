Code to publish webcam data to a rostopic. A subscriber displays this data on
the screen.
Additional ros service is added to save the current frame. The service
subscribes to the webcam data requires an optional filename parameter.
Usage for pubisher -

    rosrun webcam_ros webcam_pub

Usage for subscriber -

    rosrun webcam_ros webcam_sub

Usage for service -

    rosrun webcam_ros image_processing_server _filename:=filename.jpg

%04i can be used for numbering of image

%s can be used for default image type (default type = .jpg)
