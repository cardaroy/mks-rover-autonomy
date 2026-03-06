# in one terminal
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf --gui-render-engine ogre2"

# in another terminal
ros2 launch ros_gz_sim_demos image_bridge.launch.py image_topic:=/model/diffbot/camera/image_raw

# view image
ros2 run rqt_image_view rqt_image_view
