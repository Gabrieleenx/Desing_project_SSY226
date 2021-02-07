## OpenDLV microservice for boat controller 


## Maintainers
Creator - Gabriel Arslan Waltersson

## Table of Contents
* [MicroServise](#microservice)
* [Dependencies](#dependencies)
* [Usage](#usage)
* [Build from sources on the example of Ubuntu](#build-from-sources-on-the-example-of-ubuntu)
* [License](#license)

## MicroService

## Dependencies
No dependencies! You just need a C++14-compliant compiler to compile this project as it ships the following dependencies as part of the source distribution:

* [libcluon](https://github.com/chrberger/libcluon) - [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.txt)
* [Unit Test Framework Catch2](https://github.com/catchorg/Catch2/releases/tag/v2.1.2) - [![License: Boost Software License v1.0](https://img.shields.io/badge/License-Boost%20v1-blue.svg)](http://www.boost.org/LICENSE_1_0.txt)

## Usage
docker run --rm -ti --init --net=host  controller:latest --cid="112" --verbose --p_rudder=1 --i_rudder=0 --d_rudder=0 --p_thruster=1 --i_thruster=0 --d_thruster=0 --target_dist=2 --goal_dist=1 --freq=50  

For use on Porpoise make relevant updates to docker-compose.yml in folder usecase. 

## message communication
Inputs
- pose data
- mission (relevant for controller: target speed (m/s): mission controller (0: do nothing, 1: follow way points))
- waypoints (defined in local coordinates)

Outputs
- controll msg for prugw

## Build from sources on the example of Ubuntu
To build this software, you need cmake, C++14 or newer, and make. Having these
preconditions, just run `cmake` and `make` as follows:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make && make test && make install
```
or with docker locally 
```
docker build -t controller:latest -f Dockerfile .
```


## License
* This project is released under the terms of the GNU GPLv3 License
