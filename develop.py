import os
import docker
import docker.errors
import subprocess

def lookForDevContainer(docker_client):
    '''
    look for the dev image locally; build if it doesn't exist
    '''
    try:
        dev_container = docker_client.images.get("spot_ros2_dev_container:latest")
    except docker.errors.ImageNotFound:
        print("Image not found, building")
    finally:
        return dev_container
    
def startDevContainer(docker_client, image):
    '''
    mount current repo and sub folders containing code into the container
    '''
    current_dir = os.path.abspath(os.getcwd())
    ros_ws_path = '/ros_ws/src'
    
    # Create volume mounts for each subdirectory
    volumes = {
        current_dir: {
            'bind': ros_ws_path,
            'mode': 'rw'
        }
    }
    
    # Create the container
    container = docker_client.containers.create(
        image,
        volumes=volumes,
        command='bash',
        tty=True,
        stdin_open=True
    )
    container.start()
    return container

def buildSourceFils(running_container):
    running_container.exec_run(". /opt/ros/humble/setup.sh")

def main():
    client = docker.from_env()
    image = lookForDevContainer(client)
    container = startDevContainer(client, image)
    subprocess.run(["docker", "exec", "-it", container.id, "bash"])
    container.stop()

if __name__== "__main__":
    main()