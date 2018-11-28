# Docker

## Environment variables in docker

The `entrypoint.sh` file sets a variety of useful environment variables inside the `docker` image. They
are listed here for convenience

```
PDC_ROS_SOURCE_DIR=~/code
DATA_DIR=~/data
DC_DATA_DIR=$DATA_DIR/pdc
DC_SOURCE_DIR=$PDC_ROS_SOURCE_DIR/pytorch-dense-correspondence
```
