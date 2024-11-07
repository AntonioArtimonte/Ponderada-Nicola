from .Utils.Finders import find, is_obstructed

class Robot:
    def __init__(self, maze):
        self.maze = maze
        self.pos = find(self.maze.get_occupancy_grid(), 'r')
        self.surroundings = self.check_surroundings()

    def check_surroundings(self):
        left = self.maze.get_cell((self.pos[0], self.pos[1]-1))
        down = self.maze.get_cell((self.pos[0]+1, self.pos[1]))
        up = self.maze.get_cell((self.pos[0]-1, self.pos[1]))
        right = self.maze.get_cell((self.pos[0], self.pos[1]+1))
        return left, down, up, right

    def move(self, direction):
        new_row, new_col = self.pos
        if direction == 'left':
            new_col -= 1
        if direction == 'right':
            new_col += 1
        if direction == 'down':
            new_row += 1
        if direction == 'up':
            new_row -= 1
        new_pos = new_row, new_col
        if not is_obstructed(self.maze.get_occupancy_grid(), new_pos):
            self.maze.set_cell(self.pos, 'f')
            self.maze.set_cell(new_pos, 'r')
            self.pos = find(self.maze.get_occupancy_grid(), 'r')
            self.surroundings = self.check_surroundings()
            return True, self.surroundings
        return False, self.surroundings
