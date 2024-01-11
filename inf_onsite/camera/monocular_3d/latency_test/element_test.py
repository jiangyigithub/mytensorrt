import re
from collections import Counter

def analyze_element_values(log_filename, target_element="qtdemux0"):
    with open(log_filename, 'r') as file:
        lines = file.readlines()

    element_values = []

    for line in lines:
        # Use a regular expression to find the "element" attribute and its value
        match = re.search(r"element=\(string\)([^,]+),", line)
        # match = re.search(r"element=\(string\)(\w+),", line)

        if match:
            element_value = match.group(1)
            # exclude lines where the "element" attribute contains the placeholder "%s,"
            if "%s" not in element_value:
                element_values.append(element_value)

    # Count the occurrences of each element value
    element_counts = Counter(element_values)

    # Identify values that are not equal to the target_element
    non_target_values = [value for value in element_counts.keys() if value != target_element]

    return element_counts, non_target_values

# Example usage
log_filename = 'latency_mp4.log'
element_counts, non_target_values = analyze_element_values(log_filename)

# Print the element counts
for element, count in element_counts.items():
    print(f"Element: {element}, Count: {count}")

# Print values that are not equal to "qtdemux0"
print(f"\nValues not equal to 'qtdemux0': {non_target_values}")