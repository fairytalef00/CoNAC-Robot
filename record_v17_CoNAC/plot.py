import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_data(csv_file_path):
    """CSV 데이터를 읽고 지정된 데이터 세트를 플로팅합니다."""
    if not os.path.exists(csv_file_path):
        print(f"Error: File {csv_file_path} does not exist.")
        return

    # CSV 파일 읽기
    data = pd.read_csv(csv_file_path)

    # 기존 플롯 세트
    sets = [
        ("r1", "q1"),
        ("r2", "q2"),
        ("rdot1", "qdot1"),
        ("rdot2", "qdot2"),
        ("u1", "u2"),
        ("Vn1", "Vn2", "Vn3"),
        ("lbd1", "lbd2", "lbd3", "lbd4")
    ]

    # 플롯 저장 경로 설정
    output_dir = os.path.join(os.path.dirname(csv_file_path), "plots")
    os.makedirs(output_dir, exist_ok=True)

    # 각 세트에 대해 플롯 생성
    for i, variables in enumerate(sets):
        plt.figure(figsize=(10, 6))
        for var in variables:
            if var in data.columns:
                plt.plot(data["Time"], data[var], label=var)
            else:
                print(f"Warning: {var} not found in CSV columns.")
        plt.xlabel("Time (s)")
        plt.ylabel("Values")
        plt.title(f"Plot {i + 1}: {', '.join(variables)}")
        plt.legend()
        plt.grid(True)

        # 플롯 저장
        plot_file = os.path.join(output_dir, f"plot_{i + 1}.png")
        plt.savefig(plot_file)
        plt.close()
        print(f"Saved plot: {plot_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot.py <path_to_csv_file>")
    else:
        csv_file_path = sys.argv[1]
        plot_data(csv_file_path)
