# IMU Attitude Estimation using Unscented Kalman Filter

This project is a implementation of the paper: <br />

E. Kraft, "A quaternion-based unscented Kalman filter for orientation tracking," Sixth International Conference of Information Fusion, 2003. Proceedings of the, Cairns, QLD, Australia, 2003, pp. 47-54, doi: 10.1109/ICIF.2003.177425.


`Wrapper.py` contains functions for estimating orientations based on data from Gyroscope, Accelerometer, Complementary Filter, Madgwick Filter and Unscented Kalman Filter.<br /><br />
`helpers.py` contains the functions for quaternion processing and relating to rotations 

1. Clone this repository
2. Navigate to the folder in your terminal
3. Configure the paths to your Data in the `cfg.yaml`
4. In the terminal, run the command
```bash
python Wrapper.py
```
5. Analyse the plots and adjust the filter weights to resemble the plot represented by 'VICON'
6. Uncomment the Last section to visualize the Rotplots.

# Results

Plot of orientation angles estimated from all functions are plotted with legend. <br />
Please refer to `Report.pdf` for our results and methodology
