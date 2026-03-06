i have this code in my bashrc:
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/mksneo/mks_ws/install/mks_description/share/mks_description

export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/mksneo/mks_ws/install/mks_description/share/mks_description

i have both:
<mesh filename="model://eve/meshes/chassis.stl"/> and <mesh filename="package://mks_description/meshes/front_left_leg.stl" scale="0.001 0.001 0.001"/> in my xacro, nothing works.

my stls are in: /home/mksneo/mks_ws/src/mks_description/meshes

should i move anything, what else to try