# CRA1
## Build
`dts devel build -f --arch arm32v7 -H maxicar.local`

## Run:
`docker -H maxicar.local run -it --rm --privileged -v /data:/data -e MODE=brightness --net host duckietown/dt-duckiebot-braitenberg:arm32v7`

## Show GUI
`dts start_gui_tools maxicar --base_image duckietown/dt-core:daffy-amd64`

`rqt_image_view`