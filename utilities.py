import math
import json
from math import atan2, asin, sqrt

M_PI = 3.1415926535


class Logger:
    def __init__(self, filename, headers=None):
        if headers is None:
            headers = ["e", "e_dot", "e_int", "stamp"]
        self.filename = filename

        with open(self.filename, "w") as file:
            header_str = ""

            for header in headers:
                header_str += header
                header_str += ", "

            header_str += "\n"

            file.write(header_str)

    def log_values(self, values_list: list):
        with open(self.filename, "a") as file:
            # TODO Part 5: Write the values from the list to the file
            file.write(",".join(json.dumps(v) for v in values_list) + "\n")

    def save_log(self):
        pass


class FileReader:
    def __init__(self, filename):
        self.filename = filename

    def read_file(self):
        read_headers = False

        table = []
        headers = []
        with open(self.filename, "r") as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values = line.strip().split(",")

                    for val in values:
                        if val == "":
                            break
                        headers.append(val.strip())

                    read_headers = True
                    break

            next(file)

            # Read each line and extract values
            for line in file:
                values = line.strip().split(",")

                row = []

                for val in values:
                    if val == "":
                        break
                    row.append(float(val.strip()))

                table.append(row)

        return headers, table


# TODO Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat: list[float]):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1: # to handle gimbal lock
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw
