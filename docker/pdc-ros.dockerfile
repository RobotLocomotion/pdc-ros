ARG PARENT_IMAGE
FROM ${PARENT_IMAGE}

ARG USER_NAME
ARG USER_PASSWORD
ARG USER_ID
ARG USER_GID

RUN echo $USER_ID

RUN apt-get update
WORKDIR /home/$USER_NAME

# install ros
COPY ./install_ros.sh /tmp/install_ros.sh
RUN yes "Y" | /tmp/install_ros.sh

# change ownership of everything to our user
RUN cd $WORKDIR && chown $USER_NAME:$USER_NAME -R .

ENTRYPOINT bash -c "source ~/code/docker/entrypoint.sh && /bin/bash"
