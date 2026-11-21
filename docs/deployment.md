# Deployment

## Build the competition image

```bash
docker build -t line_follower:latest .
```

## Run (GUI via X11)

```bash
xhost +local:docker
docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix line_follower:latest
```

The container's default command is the official one-liner:

```bash
ros2 launch line_follower_bringup competition.launch.py
```

## Submission checklist

See `design.md` §15. TODO: registry address for the image, package archive command.
