# ros_packaging_example
Example repository for making a rospackage

This package demonstrates how to use:
- Classes with publishers and subscribers on them
- Launch file syntax
- How to use parameters
- How to write nodes capable of running multiple instances without interfeering with each other

To test it make sure it's built:
```
catkin_make
```

Then launch with:
```
roslaunch ros_packaging_example temperature_node.launch
```

This will launch three nodes of the same type

Each should have two different publishers.

You can check they are working with:
```
rostopic list
rostopic echo *******
```

Currently the data they send does not chance.
Check the file
```
third_party_sensor_driver.h
```
To test with new data

