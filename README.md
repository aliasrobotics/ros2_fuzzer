# ROS2 Topic & Service Fuzzer

This repository contains Alias Robotics's ROS2 topic & service fuzzer.

[![PyPI version](https://badge.fury.io/py/ros2-fuzzer.svg)](https://badge.fury.io/py/ros2-fuzzer)   
[![Documentation Status](https://readthedocs.org/projects/ros2_fuzzer/badge/?version=latest)](https://ros2_fuzzer.readthedocs.io/en/latest/?badge=latest)   


This fuzzer aims to help developers and researchers to find bugs and vulnerabilities in ROS2 nodes by performing fuzz tests
over interfaces that the target nodes process. This fuzz cases consist in pseudorandom data that is optimized to cover all
possible limit cases, in order to test the correct behaviour of the nodes against data that would otherwise not show 
in normal testing cases or normal behaviour of the target robot.

The full documentation is available on [ReadTheDocs](https://ros2-fuzzer.readthedocs.com)

ROS2_fuzzer is an effort part of the RedROS2-I FTP funded by [ROSIN](http://rosin-project.eu/) (European Union’s Horizon 2020 research and innovation programme) with the grant agreement No 732287.

<img src="http://rosin-project.eu/wp-content/uploads/2017/03/Logo_ROSIN_CMYK-Website.png"/>

Based on the original idea and Hypothesis-ROS project by [Florian Krommer](https://github.com/fkromer/hypothesis-ros)


