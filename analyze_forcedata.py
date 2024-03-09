import matplotlib.pyplot as plt

def read_data(file_path):
    times = []
    values = []
    start_time = None

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.split('\t')
            if len(parts) >= 2:
                try:
                    # Extracting time and value while removing unwanted characters
                    time = float(parts[0].replace(' s', ''))
                    value = parts[1].replace(' N', '').strip()

                    # Initialize start_time with the first valid time value
                    if start_time is None:
                        start_time = time

                    # Check if the value is a number
                    if value.replace('.', '', 1).isdigit():
                        times.append(time - start_time)  # Subtract start_time to reset time to 0
                        values.append(float(value))
                except ValueError:
                    # Skip lines that don't have proper numeric values
                    continue

    return times, values

def plot_data(file_paths):
    plt.figure(figsize=(10, 6))

    for path in file_paths:
        times, values = read_data(path)
        plt.plot(times, values, label=path.split('/')[-1])  # Label each line with the file name

    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Combined Line Chart of Data')
    plt.legend()
    plt.show()

file_paths = [
    'C:\\Users\\User\\Desktop\\Grinding Cube\\2\\小\\force data\\Fx.txt'
]

plot_data(file_paths)
