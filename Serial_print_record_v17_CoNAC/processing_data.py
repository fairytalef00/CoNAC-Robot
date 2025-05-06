import pandas as pd
import matplotlib.pyplot as plt

# 파일 경로 설정
file_path = r"C:\Users\fairytale\Desktop\COM4_2025_04_04.17.16.58.319.txt"

# 데이터 읽기
def read_data(file_path):
    # Define the correct column names based on the provided structure
    column_names = [
        "CONTROL_FLAG", "elapsedTime", "q(0)", "q(1)", "qdot(0)", "qdot(1)", 
        "r(0)", "r(1)", "rdot(0)", "rdot(1)", "u(0)", "u(1)", "u_sat(0)", 
        "u_sat(1)", "lbd(0)", "lbd(1)", "lbd(2)", "lbd(3)", "lbd(4)", 
        "lbd(5)", "lbd(6)", "lbd(7)", "Vn(0)", "Vn(1)", "Vn(2)", "avgCtrlTimesec"
    ]
    try:
        # Read the file while skipping malformed rows
        data = pd.read_csv(
            file_path, 
            sep=r'\s+',  # Use raw string to avoid SyntaxWarning
            engine='python',
            names=column_names,  # Use predefined column names
            skiprows=1,  # Skip the first row if it contains invalid headers
            on_bad_lines='skip'  # Skip malformed lines
        )
        # Validate the number of columns in each row
        if len(data.columns) != len(column_names):
            print(f"Error: Expected {len(column_names)} columns, but got {len(data.columns)}.")
            return None
        print("Columns in the dataset:", data.columns.tolist())  # Debug: Print column names
        print("First few rows of the dataset:\n", data.head())  # Debug: Print first few rows
        return data
    except pd.errors.ParserError as e:
        print(f"Error parsing the file: {e}")
        print("Attempting to debug the file structure...")
        with open(file_path, 'r') as file:
            for i, line in enumerate(file):
                if i == 2068 or i == 2069:  # Lines around the reported error
                    print(f"Line {i + 1}: {line.strip()}")
        return None

# 데이터 필터링
def filter_data(data):
    # Check if 'CONTROL_FLAG' column exists
    if "CONTROL_FLAG" not in data.columns:
        print("Error: 'CONTROL_FLAG' column is missing from the dataset.")
        print("Available columns:", data.columns.tolist())  # Debug: Print available columns
        return None
    # CONTROL_FLAG 열이 1인 경우만 필터링
    filtered_data = data[data["CONTROL_FLAG"] == 2]
    return filtered_data

# 플롯 생성
def plot_data(data):
    # x축: elapsedTime
    x = data["elapsedTime"]

    # 플롯 설정
    plt.figure(figsize=(15, 15))  # Increase figure size for a larger grid

    # 1. set(q(0), r(0))
    plt.subplot(5, 2, 1)
    plt.plot(x, data["q(0)"], label="q(0)")
    plt.plot(x, data["r(0)"], label="r(0)")
    plt.xlabel("elapsedTime")
    plt.ylabel("q(0), r(0)")
    plt.title("Set (q(0), r(0))")
    plt.legend()
    plt.grid()

    # 2. set(q(1), r(1))
    plt.subplot(5, 2, 2)
    plt.plot(x, data["q(1)"], label="q(1)")
    plt.plot(x, data["r(1)"], label="r(1)")
    plt.xlabel("elapsedTime")
    plt.ylabel("q(1), r(1)")
    plt.title("Set (q(1), r(1))")
    plt.legend()
    plt.grid()

    # 3. set(qdot(0), rdot(0))
    plt.subplot(5, 2, 3)
    plt.plot(x, data["qdot(0)"], label="qdot(0)")
    plt.plot(x, data["rdot(0)"], label="rdot(0)")
    plt.xlabel("elapsedTime")
    plt.ylabel("qdot(0), rdot(0)")
    plt.title("Set (qdot(0), rdot(0))")
    plt.legend()
    plt.grid()

    # 4. set(qdot(1), rdot(1))
    plt.subplot(5, 2, 4)
    plt.plot(x, data["qdot(1)"], label="qdot(1)")
    plt.plot(x, data["rdot(1)"], label="rdot(1)")
    plt.xlabel("elapsedTime")
    plt.ylabel("qdot(1), rdot(1)")
    plt.title("Set (qdot(1), rdot(1))")
    plt.legend()
    plt.grid()

    # 5. set(lbd(0), ..., lbd(7))
    plt.subplot(5, 2, 5)
    for i in range(8):
        plt.plot(x, data[f"lbd({i})"], label=f"lbd({i})")
    plt.xlabel("elapsedTime")
    plt.ylabel("lbd(0) ... lbd(7)")
    plt.title("Set (lbd(0) ... lbd(7))")
    plt.legend()
    plt.grid()

    # 6. set(Vn(0), ..., Vn(2))
    plt.subplot(5, 2, 6)
    for i in range(3):
        plt.plot(x, data[f"Vn({i})"], label=f"Vn({i})")
    plt.xlabel("elapsedTime")
    plt.ylabel("Vn(0) ... Vn(2)")
    plt.title("Set (Vn(0) ... Vn(2))")
    plt.legend()
    plt.grid()

    # 7. avgCtrlTimesec
    plt.subplot(5, 2, 7)
    plt.plot(x, data["avgCtrlTimesec"], label="avgCtrlTimesec")
    plt.xlabel("elapsedTime")
    plt.ylabel("avgCtrlTimesec")
    plt.title("Average Control Time (sec)")
    plt.legend()
    plt.grid()

    # 8. set(u(0), u_sat(0))
    plt.subplot(5, 2, 9)
    plt.plot(x, data["u(0)"], label="u(0)")
    plt.plot(x, data["u_sat(0)"], label="u_sat(0)")
    plt.xlabel("elapsedTime")
    plt.ylabel("u(0), u_sat(0)")
    plt.title("Set (u(0), u_sat(0))")
    plt.legend()
    plt.grid()

    # 9. set(u(1), u_sat(1))
    plt.subplot(5, 2, 10)
    plt.plot(x, data["u(1)"], label="u(1)")
    plt.plot(x, data["u_sat(1)"], label="u_sat(1)")
    plt.xlabel("elapsedTime")
    plt.ylabel("u(1), u_sat(1)")
    plt.title("Set (u(1), u_sat(1))")
    plt.legend()
    plt.grid()

    # 플롯 표시
    plt.tight_layout()
    plt.show()

# 메인 함수
def main():
    # 데이터 읽기
    data = read_data(file_path)
    if data is None:
        print("Failed to read data. Exiting.")
        return

    # 데이터 필터링
    filtered_data = filter_data(data)
    if filtered_data is None:
        print("Failed to filter data. Exiting.")
        return

    # 플롯 생성
    plot_data(filtered_data)

if __name__ == "__main__":
    main()