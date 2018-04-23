import math


class Path:
    def __init__(self, path, x_start, y_start):
        self.base_path = path
        self.angles = []
        self.path = []
        self.error = float(0)
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
        #  if end_index > len(self.path[0][:])-1:
        #   end_index = len(self.path[0][:])-1
        angle = self.angles[error_index]
        expected = [self.path[0][error_index], self.path[1][error_index]]
        error = self.cross_track_error(expected, actual, angle)  # Sets an initial value
        for i in range(error_index, end_index):
            if i > len(self.angles)-1:
                index = i - (len(self.angles)-1)
            else:
                index = i
            angle = self.angles[index]
            expected = [self.path[0][index], self.path[1][index]]
            temp_error = self.cross_track_error(expected, actual, angle)
            if abs(temp_error) < abs(error):
                error_index = index
                error = temp_error
        # If error is reduced it will advance your reference position to the smallest error found
        self.error_index = error_index
        print(error_index)
        if self.error_index > len(self.angles)-1:
            # If you reach the end, Let it keep going around the loop
            self.error_index = 0
            print('Went around the loop')
        self.error = error
        return self.error

    def cross_track_error(self, expected, actual, angle):
        # Cross track error reference
        # https://brage.bibsys.no/xmlui/bitstream/handle/11250/280167/FossenPettersenGaleazzi2014.pdf?sequence=3
        angle = angle
        error = -(actual[0] - expected[0])*math.sin(angle) + (actual[1] - expected[1])*math.cos(angle)
        return float(error)

    def reset(self, x_start, y_start):
        # Resets the path for a new trial and updates the planned path based on your starting position
        self.error_index = 0
        self.error = float(0)

        # update path based on starting position
        self.path = [[x + x_start for x in self.base_path[0][:]], [y + y_start for y in self.base_path[1][:]]]
        self.find_path_tangent_angles()
        print('start')
        print(x_start)
        print(y_start)
