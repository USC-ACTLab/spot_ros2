import os
import subprocess

import docker
import docker.errors


def lookForDevContainer(docker_client):
    """
    look for the dev image locally; build if it doesn't exist
    """
    dev_image_label = "spot_ros2_dev_container:latest"
    try:
        dev_container = docker_client.images.get(dev_image_label)
    except docker.errors.ImageNotFound:
        print("Image not found, building")
        curr_dir = os.path.abspath(os.getcwd())
        dev_container, build_log = docker_client.images.build(
            path=curr_dir, dockerfile=os.path.join(curr_dir, "devContainer.Dockerfile"), tag=dev_image_label, rm=True
        )
    finally:
        return dev_container


def startDevContainer(docker_client, image):
    """
    mount current repo and sub folders containing code into the container
    """
    current_dir = os.path.abspath(os.getcwd())
    ros_ws_path = "/ros_ws/src"

    # Create volume mounts for each subdirectory
    volumes = {current_dir: {"bind": ros_ws_path, "mode": "rw"}}

    # Create the container
    container = docker_client.containers.create(image, volumes=volumes, command="bash", tty=True, stdin_open=True)
    container.start()
    return container


def buildSourceFils(running_container):
    running_container.exec_run("source /opt/ros/humble/setup.bash")


def main():
    client = docker.from_env()
    image = lookForDevContainer(client)
    container = startDevContainer(client, image)
    buildSourceFils(container)
    subprocess.run(["docker", "exec", "-it", container.id, "bash"])
    container.stop()


if __name__ == "__main__":
    main()
