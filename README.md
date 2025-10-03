# SAM2 x ROS2   
This package contains ROS2 packages for utilizing Metas SAM2 model as a service call.  
Additionally, this package contains a Dockerfile which builds a container solely responsible for hosting a SAM2 service node. The docker image uses a ROS2 Jazzy image.  
  
## Setup  
### Using Docker   
To build and run the Docker container, simply call the run_container script with the build flag (-b).  
Upon startup, the script *docker_startup.sh* will be executed, automatically starting the ROS node. This script also sets the ROS_DOMAIN_ID, and can be modified as needed, the container will just need to be partially rebuilt after making changes.  
```  
cd sam2_ros    
chmod +x run_container.sh  
./run_container.sh -b  
```  
  
### Using a local ROS2 Workspace  
1. Clone the (SAM2 github repository)[https://github.com/facebookresearch/sam2], and follow the instructions for installing dependencies and downloading model checkpoints.  
```  
git clone https://github.com/facebookresearch/sam2.git && cd sam2  
pip install -e .  
cd checkpoints && ./download_ckpts.sh && cd ..  
```  
2. Clone this repository into your ROS workspace.  
```  
```     
3. Edit the SAM2_DIR on line 15 of server_node.py (sam2_server/sam2_server/server_node.py) to be the full local filepath of wherever SAM2 was cloned.    
  
## Usage   
Once the packages are built and the ROS workspace has been sourced, the SAM2 server node can be run like so:   
```  
ros2 run sam2_server server  
```      
  
The server node then hosts a service named '/segment' which accepts an input image, mode (0=point prommpt, 1=bounding box prompt), and either a list of points or bounding boxes to segment (x1,y1,x2,y2).  
  
For example usage of service calls within a python script, check out this script (which can also be run directly to test the functionality of the service):  
```  
sam2_server/test/sam_service_test.py   
```    
