from Kalman_filter import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
from bagpy import bagreader
import pandas as pd

kf_x, kf_y = [], []
gps_x, gps_y = [], []

kf = KalmanFilter(
    initial_state=np.zeros(5),
    initial_covariance=np.eye(5) * 0.1,
    process_noise=np.eye(5) * 0.01,  # to be tuned
    gps_noise=np.eye(2) * 4.0 # to be tuned
)

last_time = None
state = np.zeros(5)


def get_speedsteer_data(speedsteer_df, time):
    """Getting the data from /speedsteer topic"""
    global last_time
    filtered = speedsteer_df[speedsteer_df['Time'] >= time]
    if filtered.empty:
        return None, None, None, 0  

    row = filtered.iloc[0]
    current_time = row['Time']
    
    if last_time is None:
        last_time = current_time
        return None, None, None, 0

    dt = current_time - last_time
    v = row['point.y'] / 3.6  
    steering_angle_deg = row['point.x']
    alpha = np.radians(steering_angle_deg)
    wheelbase = 1.765
    last_time = current_time
    return v, alpha, wheelbase, dt

def run_ekf():
    """Main function to run the Kalman Filter"""
    global last_time, state, gps_x, gps_y, kf_x, kf_y

    gps_x, gps_y, kf_x, kf_y = [], [], [], []

    # loading the bag files
    try:
        br1 = bagreader('/Users/ortol/Desktop/Polimi/Robotics/Robotics_Project/data/project.bag')
        br2 = bagreader('/Users/ortol/Desktop/Polimi/Robotics/Robotics_Project/data/my_drive.bag')
        
        # retrieving the messages from each topic and converting them into dataframes
        speedsteer_df = pd.read_csv(br1.message_by_topic('/speedsteer'))
        odom_df = pd.read_csv(br2.message_by_topic('/odom'))
        gps_df = pd.read_csv(br2.message_by_topic('/gps_odom'))
    except Exception as e:
        print(f"⚠️ Failed to load bag files: {str(e)}")
        return

    # Convert and sort timestamps
    try:
        odom_df['Time'] = odom_df['Time'].astype(float)
        gps_df['Time'] = gps_df['Time'].astype(float)
        speedsteer_df['Time'] = speedsteer_df['Time'].astype(float)
        
        # FIX: Time-shift speedsteer data to match odom/gps -> logs just for debugging
        print("\nBefore time adjustment:")
        print(f"First SpeedSteer time: {speedsteer_df['Time'].iloc[0]}")
        print(f"First Odom time: {odom_df['Time'].iloc[0]}")
        
        # computation of the time offset (difference between first messages)
        time_offset = odom_df['Time'].iloc[0] - speedsteer_df['Time'].iloc[0]
        speedsteer_df['Time'] += time_offset
        
        print("\nAfter time adjustment:")
        print(f"First SpeedSteer time: {speedsteer_df['Time'].iloc[0]}")
        print(f"First Odom time: {odom_df['Time'].iloc[0]}")
        
        odom_df = odom_df.sort_values('Time')
        gps_df = gps_df.sort_values('Time')
        speedsteer_df = speedsteer_df.sort_values('Time')
    except Exception as e:
        print(f"⚠️ Time conversion failed: {str(e)}")
        return

    # Verify data ranges
    print("\n=== Data Ranges ===")
    print(f"Odom: {len(odom_df)} points, {odom_df['Time'].min():.2f} to {odom_df['Time'].max():.2f}")
    print(f"GPS: {len(gps_df)} points, {gps_df['Time'].min():.2f} to {gps_df['Time'].max():.2f}")
    print(f"SpeedSteer: {len(speedsteer_df)} points, {speedsteer_df['Time'].min():.2f} to {speedsteer_df['Time'].max():.2f}")

    # Check for data overlap
    time_overlap = min(odom_df['Time'].max(), gps_df['Time'].max()) - max(odom_df['Time'].min(), gps_df['Time'].min())
    if time_overlap <= 0:
        print("⚠️ No time overlap between Odom and GPS data!")
        return
    print(f"Time overlap: {time_overlap:.2f} seconds")

    # Initialize Kalman Filter with first valid state
    try:
        first_odom = odom_df.iloc[0]
        initial_state = np.array([
            first_odom['pose.pose.position.x'],
            first_odom['pose.pose.position.y'],
            np.radians(first_odom['pose.pose.position.z']),
            0,  # Initial velocity
            0   # Initial angular velocity
        ])
        kf = KalmanFilter(
            initial_state=initial_state,
            initial_covariance=np.eye(5) * 0.1,
            process_noise=np.eye(5) * 0.01,
            gps_noise=np.eye(2) * 0.1
        )
    except Exception as e:
        print(f"⚠️ Kalman Filter initialization failed: {str(e)}")
        return

    # Main processing loop
    last_time = None
    gps_index = 0

    for odom_idx, odom_row in odom_df.iterrows():
        current_time = odom_row['Time']
        
        # Get corresponding speed/steering data (now time-aligned)
        speedsteer_idx = speedsteer_df['Time'].searchsorted(current_time)
        if speedsteer_idx >= len(speedsteer_df):
            continue
            
        speedsteer_row = speedsteer_df.iloc[speedsteer_idx]
        dt = speedsteer_row['Time'] - last_time if last_time is not None else 0.01
        
        try:
            v = speedsteer_row['point.y'] / 3.6  # km/h to m/s
            steering_angle_deg = speedsteer_row['point.x']
            alpha = np.radians(steering_angle_deg)
            wheelbase = 1.765
            omega = v * np.tan(alpha) / wheelbase
            
            # Create state vector
            current_state = np.array([
                odom_row['pose.pose.position.x'],
                odom_row['pose.pose.position.y'],
                np.radians(odom_row['pose.pose.position.z']),
                v,
                omega
            ])
            

            # Prediction step
            kf.predict(dt, current_state)
            
            # Find matching GPS data
            while gps_index < len(gps_df) and gps_df.iloc[gps_index]['Time'] < current_time - 0.1:
                gps_index += 1
                
            if gps_index < len(gps_df) and abs(gps_df.iloc[gps_index]['Time'] - current_time) <= 0.1:
                gps_measurement = np.array([
                    gps_df.iloc[gps_index]['pose.pose.position.x'],
                    gps_df.iloc[gps_index]['pose.pose.position.y']
                ])

                if np.array_equal(gps_measurement, [-1028663.637769047, -4477422.006266854]):
                    gps_measurement = gps_df.iloc[gps_index - 1][['pose.pose.position.x', 'pose.pose.position.y']].values

                kf.update(gps_measurement)
                gps_x.append(gps_measurement[0])
                gps_y.append(gps_measurement[1])
                gps_index += 1
                
            # Store results
            state = kf.get_state()
            kf_x.append(state[0])
            kf_y.append(state[1])
            print(f"Time: {current_time:.2f}, Kalman State: {state}, GPS: {gps_measurement if gps_index < len(gps_df) else 'N/A'}")
            last_time = current_time
            
        except Exception as e:
            print(f"⚠️ Error processing data at time {current_time:.2f}: {str(e)}")
            continue

    # Results analysis and enhanced plotting
    print("\n=== Processing Results ===")
    print(f"Processed {len(kf_x)} Kalman points")
    print(f"Used {len(gps_x)} GPS updates")
    
    if len(kf_x) > 0 and len(gps_x) > 0:
        # Create a figure with 4 subplots
        plt.figure(figsize=(16, 12))
        
        # 1. Trajectory Comparison
        plt.subplot(2, 2, 1)
        plt.plot(kf_x, kf_y, 'r-', label='Kalman Filter', linewidth=2)
        plt.plot(gps_x, gps_y, 'b.', label='GPS Measurements', markersize=4, alpha=0.5)
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Trajectory Comparison')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # 2. X Position Over Time
        plt.subplot(2, 2, 2)
        time_axis = np.arange(len(kf_x)) * 0.1  # Assuming ~10Hz data
        plt.plot(time_axis, kf_x, 'r-', label='Kalman X')
        plt.plot(time_axis[:len(gps_x)], gps_x, 'b.', label='GPS X', markersize=4)
        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.title('X Position Comparison')
        plt.legend()
        plt.grid(True)
        
        # 3. Y Position Over Time
        plt.subplot(2, 2, 3)
        plt.plot(time_axis, kf_y, 'g-', label='Kalman Y')
        plt.plot(time_axis[:len(gps_y)], gps_y, 'c.', label='GPS Y', markersize=4)
        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.title('Y Position Comparison')
        plt.legend()
        plt.grid(True)
        
        # 4. Position Error
        plt.subplot(2, 2, 4)
        min_len = min(len(kf_x), len(gps_x))
        error_x = [abs(kf_x[i] - gps_x[i]) for i in range(min_len)]
        error_y = [abs(kf_y[i] - gps_y[i]) for i in range(min_len)]
        plt.plot(time_axis[:min_len], error_x, 'm-', label='X Error')
        plt.plot(time_axis[:min_len], error_y, 'y-', label='Y Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Position Error (m)')
        plt.title('Estimation Error')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()

        # Additional plot: Innovation sequence (GPS measurement residuals)
        if len(kf_x) == len(gps_x):
            plt.figure(figsize=(12, 6))
            residuals_x = [gps_x[i] - kf_x[i] for i in range(len(gps_x))]
            residuals_y = [gps_y[i] - kf_y[i] for i in range(len(gps_y))]
            plt.plot(residuals_x, 'r-', label='X Residual')
            plt.plot(residuals_y, 'b-', label='Y Residual')
            plt.xlabel('Measurement Index')
            plt.ylabel('Residual (m)')
            plt.title('Innovation Sequence (Measurement Residuals)')
            plt.legend()
            plt.grid(True)
            plt.show()
    else:
        # Additional debugging prints
        print("⚠️ Still no data processed after time adjustment!")
        print("Possible solutions:")
        print("1. Check if speedsteer data is actually synchronized with odom")
        print("2. Try using twist.twist.linear.x from odom as velocity source")
        print("3. Verify all topics have the expected data structure")
run_ekf()
