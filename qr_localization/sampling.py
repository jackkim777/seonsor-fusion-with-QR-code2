import csv

def reduce_csv(input_file, output_file, num_samples=100):
    with open(input_file, 'r', newline='') as infile:
        reader = list(csv.reader(infile))
        header = reader[0]  # 헤더 읽기
        rows = reader[1:]  # 데이터 부분만 추출
        total_rows = len(rows)
        print(total_rows // (num_samples - 1))
        interval = max(1, total_rows // (num_samples - 1))  # 샘플링 간격 계산
        interval = 5
        with open(output_file, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(header)  # 헤더 쓰기

            # 각 간격마다 라인을 선택하여 쓰기
            sampled_rows = [rows[i] for i in range(0, total_rows, interval)]
            # 101개의 샘플을 보장하기 위해 마지막 라인을 추가
            if len(sampled_rows) < num_samples:
                sampled_rows.append(rows[-1])

            writer.writerows(sampled_rows[:num_samples])

input_csv = 'trajectory.csv'
output_csv = 'sampled_trajectory.csv'
reduce_csv(input_csv, output_csv)