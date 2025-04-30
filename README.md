# depthimage_to_pointcloud
ROS node that crops a depthimage and publishes it as a pointcloud2

Subscribes to depthimage at `/camera/depth/image_rect_raw` and publishes to `/camera/depth/points_cropped/`. These can be changed by modifying `depthimage_crop_to_pointcloud.py`.

Add files to your catkin workspace and build. 
