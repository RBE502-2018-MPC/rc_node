import math


class Path:
    def __init__(self, path, x_start, y_start):
        self.base_path = path
        self.angles = []
        self.path = []
        self.error = 0
        self.error_index = 0
        self.reset(x_start, y_start)

    def find_path_tangent_angles(self):
        angles = []
        for i in range(0, len(self.path[0][:]) - 2):
            x1 = self.path[0][i]
            x2 = self.path[0][i+1]
            y1 = self.path[1][i]
            y2 = self.path[1][i+1]
            angle = math.atan2((y2-y1), (x2-x1))
            angles.append(angle)
        self.angles = angles

    def find_error(self, actual):
        # Reset error calculation parameters
        error_index = self.error_index + 1  # chooses the next path position so we don't get stuck on one
        end_index = error_index + 100  # Using a resolution of 0.0001 m per step, lower this to reduce computation

        if end_index > len(self.path[0][:])-1:
            end_index = len(self.path[0][:])-1
        angle = self.angles[len(self.angles) - 1]
        expected_end = [self.path[0][error_index], self.path[1][error_index]]
        error = self.cross_track_error(expected_end, actual, angle)  # Sets an initial value
        for i in range(error_index, end_index):
            expected_end = [self.path[0][error_index], self.path[1][error_index]]
            if error_index > len(self.angles)-1:
                angle = self.angles[len(self.angles)-1]
            else:
                angle = self.angles[error_index]
            temp_error = self.cross_track_error(expected_end, actual, angle)
            if abs(temp_error) < abs(error):
                error_index = i
                error = temp_error
        # If error is reduced it will advance your reference position to the smallest error found
        self.error_index = error_index
        self.error = error
        return self.error, self.error_index

    def cross_track_error(self, expected, actual, angle):
        # Cross track error reference
        # https://brage.bibsys.no/xmlui/bitstream/handle/11250/280167/FossenPettersenGaleazzi2014.pdf?sequence=3
        angle = angle
        error = -(actual[0] - expected[0])*math.sin(angle) + (actual[1] - expected[1])*math.cos(angle)
        return error

    def reset(self, x_start, y_start):
        # Resets the path for a new trial and updates the planned path based on your starting position
        self.error_index = 0
        self.error = 10000

        # update path based on starting position
        self.path = [[x + x_start for x in self.base_path[0][:]], [y + y_start for y in self.base_path[1][:]]]
        self.find_path_tangent_angles()
