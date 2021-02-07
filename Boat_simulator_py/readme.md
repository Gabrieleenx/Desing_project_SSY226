## Porpoise boat simulator

## Maintainers
Creator - Gabriel Arslan Waltersson 

## Simulator
It's a simple force based simulator for the a boat with 2 individual thrusters, 2 rudders and 1 bow truster. Both water and wind disturbances can be modeled in the main script if desierd. When running its ploting the boat and the path it has taken and the target path. 

## Dependencies
linux

You need to install libcluon
```
sudo add-apt-repository ppa:chrberger/libcluon
sudo apt-get update
sudo apt-get install libcluon
```

And packages - for only simulator
```
sudo apt-get install --no-install-recommends \
    build-essential \
    python3-protobuf \
    python3-sysv-ipc \
    python3-numpy \
    python3-opencv \
    protobuf-compiler \
    python3-matplotlib \
    python3-pip \
    python3-tk
```
Extra packages for sim_system 
```
pip3 install utm==0.7.0
```

 ## Run
To run the stenpiren to lindholem test, start the controller and then start the simulator_stenpiren.py file. Changes can be made to the dile without recompiling the libcluon parts. The simulator will start att steniren and then in realtime go to lindholmen. Different noice and disturbance settings can be cahanges in the file. There will be a live plot updating once a second, and when the target has been reached, results will be printed in the terminal. To stop once done just close the figure. 
```
python3 simulator_stenpiren.y
```

 To run as is or only changes made to python file then run (start controller before simulator) from main folder. This tests only the controller on a samller map.
```
python3 simulator.py
```
For testing larger parts of the system run. (make sure contorller and filter is running) 
```
python3 sim_system.py
``` 
## Rebuild libcluon for pyhon 
Only needed if changes to messages or other probelms, enter folder and run
```
make
```














































