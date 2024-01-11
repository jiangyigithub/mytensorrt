import re
from collections import defaultdict
import matplotlib.pyplot as plt


def analyze_time_values_by_element(log_filename):
    with open(log_filename, 'r') as file:
        lines = file.readlines()

    time_values_by_element = defaultdict(list)

    for line in lines:
        # Use a regular expression to find the "element" and "time" attributes and their values
        element_match = re.search(r"element=\(string\)([^,]+),", line)
        time_match = re.search(r"time=\(guint64\)(\d+),", line)

        if element_match and time_match :
            element_value = element_match.group(1)
            time_value_ns = int(time_match.group(1))
            time_value_ms = time_value_ns / 1e6  # Convert nanoseconds to milliseconds
            time_values_by_element[element_value].append(time_value_ms)

            # Break the loop if the maximum samples is reached for the first element
            if len(time_values_by_element[element_value]) == 70:
                    break

    # Calculate statistics for each element
    element_statistics = {element: {"mean": sum(times) / len(times), "count": len(times)} for element, times in time_values_by_element.items()}

    return element_statistics, time_values_by_element

# Example usage
log_filename = 'latency_mp4.log'
element_statistics, time_values_by_element = analyze_time_values_by_element(log_filename)

# Print the element-wise statistics
for element, stats in element_statistics.items():
    print(f"Element: {element}, Mean Time: {stats['mean']}, Count: {stats['count']}")

# Plot the time values for each element
for element, times in time_values_by_element.items():
    if(element=='primary-inference'):
        plt.plot(times, label=element)

plt.xlabel('Sample Index')
plt.ylabel('Time')
plt.title('Time values for each element')
plt.legend()
plt.show()