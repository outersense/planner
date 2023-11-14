import math

def calculate_new_yaw(input_file, output_file):
    def read_points(file_path):
        with open(file_path, 'r') as file:
            return [list(map(float, line.split())) for line in file]

    def write_points_with_new_yaw(points, file_path):
        with open(file_path, 'w') as file:
            for point in points:
                file.write(f"{point[0]} {point[1]} {point[2]}\n")

    def compute_yaw(p1, p2):
        return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    points = read_points(input_file)

    for i in range(1, len(points)):
        points[i][2] = compute_yaw(points[i - 1], points[i])

    write_points_with_new_yaw(points, output_file)

# Example usage
calculate_new_yaw('Nov10_manual_jash_100scale2.txt', 'new_Nov10_manual_jash_100scale2.txt')
