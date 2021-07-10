# Docker documentation.
## Building and running the Docker image
* Download the nvidia driver [runfile](https://www.nvidia.com/en-in/drivers/unix/) for your driver version (find using `nvidia-smi`), and place in the same dir as `Dockerfile`, call it `nvidia-driver-runfile.run`.
* Download the [pycharm .tar.gz](https://www.jetbrains.com/pycharm/download/#section=linux) in the same directory. Name it according to line 4 in `Dockerfile`.
* Modify the dockerfile, change the user's uid, gid, user and password, find uid and gid from host. Build the image using `docker build -t aayushf/ros_webots:1.0 .`.
* To start the container, run `xhost +"local:docker@"`, then run `dock.fish`.
  * Change all the directory mappings according to your project structure. 
  * Create all the directories that you mount before running the container for the first time.
* If all goes well, PyCharm should open from within the container, and you can start coding!