# Structure-Light sensor calibration 

**Update on 2022/01/08.**  


![overview](doc/principle.jpg)

### lib
You need to inatall opencv first
    ```
    brew inatall opencv

    ```

### make
Fellow steps below to run the code:

0. Configure your dataset path in ./calib_picture and ./light_picture

1. To train LSDNN using our dataset and ResNet as backbone:
    ```Shell
    cmake .
    make
    ```

### run
After make you can run by:
    ```
    ./target
    
    ```

### result
You can find the calib result in Parameters.xml

Pointcloud in laser atripe will be saves in pointcloud.txt