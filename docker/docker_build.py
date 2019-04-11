#!/usr/bin/env python
from __future__ import print_function

import argparse
import os
import getpass

if __name__ == "__main__":

    cwd = os.getcwd()
    if not os.path.isfile(os.path.join(cwd, 'docker_build.py')):
        raise RuntimeError("You must run docker_buidl.py from the /docker directory")

    pdc_ros_source_dir = os.path.join(cwd, os.pardir)
    pdc_source_dir = os.path.join(pdc_ros_source_dir, 'pytorch-dense-correspondence')

    print("building docker container . . . ")
    user_name = getpass.getuser()
    default_image_name = user_name + "-pdc-ros"

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image", type=str,
                        help="name for the newly created docker image", default=default_image_name)

    parser.add_argument("-d", "--dry_run", action='store_true',
                        help="(optional) perform a dry_run, print the command that would have been executed but don't execute it.")

    parser.add_argument("-p", "--password", type=str,
                        help="(optional) password for the user", default="password")

    parser.add_argument('-uid', '--user_id', type=int, help="(optional) user id for this user", default=os.getuid())
    parser.add_argument('-gid', '--group_id', type=int, help="(optional) user gid for this user", default=os.getgid())

    args = parser.parse_args()
    print("building docker image named ", args.image)
    pdc_image_name = user_name + "-pdc-base-image"


    # print("command = \n \n", cmd)
    # print("")



    def build_pdc_docker_container():
        """
        Builds the pdc docker container which the pdc-ros
        docker container will derive from
        """
        os.chdir(os.path.join(pdc_source_dir, 'docker'))
        # pdc_image_name = user_name + "-pdc-base-image"
        cmd = "./docker_build.py -i %(pdc_image_name)s" % {'pdc_image_name': pdc_image_name}
        print("pdc base image build command = \n \n", cmd)
        os.system(cmd)


    def build_pdc_ros_docker_container():
        os.chdir(os.path.join(pdc_ros_source_dir, 'docker'))
        cmd = "docker build " \
              "--build-arg PARENT_IMAGE=%(parent_image)s " \
              "--build-arg USER_NAME=%(user_name)s " \
              "--build-arg USER_PASSWORD=%(password)s " \
              "--build-arg USER_ID=%(user_id)s " \
              "--build-arg USER_GID=%(group_id)s " \
              % {'user_name': user_name, 'password': args.password, 'user_id': args.user_id, 'group_id': args.group_id,
                 'parent_image': pdc_image_name}

        cmd += " -t %s -f pdc-ros.dockerfile ." % args.image
        print("pdc-ros image build command = \n \n", cmd)
        os.system(cmd)


    build_pdc_docker_container()
    build_pdc_ros_docker_container()

    # build the docker image
    if not args.dry_run:
        print("executing shell command")
    # os.system(cmd)
    else:
        print("dry run, not executing command")
