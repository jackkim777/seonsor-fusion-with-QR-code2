import numpy as np
import pandas as pd

def load_trajectory(file_path):
    """
    Load trajectory data from a CSV file.
    The file should contain columns: 'x', 'y', 'z'.
    """
    data = pd.read_csv(file_path)
    if not {'x', 'y', 'z'}.issubset(data.columns):
        raise ValueError("CSV file must contain 'x', 'y', 'z' columns.")
    return data[['x', 'y', 'z']].to_numpy()

def calculate_ate(gt_trajectory, pred_trajectory):
    """
    Calculate the Absolute Trajectory Error (ATE) between the ground truth
    and predicted trajectories.
    """
    if gt_trajectory.shape != pred_trajectory.shape:
        raise ValueError("Ground truth and predicted trajectories must have the same shape.")
    
    # Calculate the Euclidean distance between each point
    differences = gt_trajectory - pred_trajectory
    squared_errors = np.sum(differences ** 2, axis=1)
    mean_squared_error = np.mean(squared_errors)
    ate = np.sqrt(mean_squared_error)
    
    return ate

def main():
    # Load trajectories from CSV files
    gt_file = 'gt_4m_case3.csv'  # Replace with your ground truth file path
    pred_file = '4m_case3.csv'  # Replace with your predicted trajectory file path
    gt_trajectory = load_trajectory(gt_file)
    pred_trajectory = load_trajectory(pred_file)
    print(gt_trajectory.shape, pred_trajectory.shape)
    # Calculate ATE
    ate = calculate_ate(gt_trajectory, pred_trajectory)
    print(f'Calculated ATE: {ate:.4f}')

if __name__ == '__main__':
    main()