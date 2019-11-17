# AMoD RH4
## Stop watchtower
`dts devel watchtower stop -H maxicar.local`

## Build
`dts devel build -f --arch arm32v7 -H maxicar.local`

## Run:
`docker -H maxicar.local run -it --rm --privileged -v /data:/data -e MODE=avoid --net host duckietown/dt-duckiebot-braitenberg:v1-arm32v7`

### Access ROS-enabled container
`docker -H maxicar.local run -it --rm --net host duckietown/dt-ros-commons:daffy-arm32v7 /bin/bash`

`rostopic list`

## Show GUI
`dts start_gui_tools maxicar --base_image duckietown/dt-core:daffy-amd64`

`rqt_image_view`