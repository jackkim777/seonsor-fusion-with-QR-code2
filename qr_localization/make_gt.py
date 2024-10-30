import numpy as np
import pandas as pd

def calculate_line_coordinates(point_a, point_b, num_points=274):
    """
    두 점 사이의 직선 좌표를 계산합니다.
    
    :param point_a: 첫 번째 점의 좌표 (x, y, z)
    :param point_b: 두 번째 점의 좌표 (x, y, z)
    :param num_points: 직선 위에 생성할 점의 개수
    :return: 직선 위의 점들의 좌표 리스트 [(x1, y1, z1), (x2, y2, z2), ...]
    """
    # 점 A와 B의 좌표를 numpy 배열로 변환
    point_a = np.array(point_a)
    point_b = np.array(point_b)
    
    # 두 점 사이를 num_points 개수로 나누어 선형 보간 (interpolation)
    coordinates = [tuple(point_a + t * (point_b - point_a)) for t in np.linspace(0, 1, num_points)]
    
    return coordinates

def save_coordinates_to_csv(coordinates, file_name='line_coordinates.csv'):
    """
    좌표 리스트를 CSV 파일로 저장합니다.
    
    :param coordinates: 직선 위의 점들의 좌표 리스트 [(x1, y1, z1), ...]
    :param file_name: 저장할 CSV 파일 이름
    """
    df = pd.DataFrame(coordinates, columns=['x', 'y', 'z'])
    df.to_csv(file_name, index=False)
    print(f'Saved coordinates to {file_name}')

# 예제 사용법
point_a = (-0.0431329, 0.0977825, 0)  # 첫 번째 점의 좌표 (x, y, z)
point_b = (19.628125,1.53880675,0.0)  # 두 번째 점의 좌표 (x, y, z)
line_coordinates = calculate_line_coordinates(point_a, point_b)

# 좌표를 CSV 파일로 저장
save_coordinates_to_csv(line_coordinates, 'gt_4m_case3.csv')