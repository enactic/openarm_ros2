# Docker GUI Forwarding

On Linux:
```sh
host +local:root
```

```sh
docker build -t ghcr.io/thomasonzhou/openarm:v0.3 .
```

```sh
docker run --env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--network=host \
-it ghcr.io/thomasonzhou/openarm:v0.3 \
/bin/bash
```
