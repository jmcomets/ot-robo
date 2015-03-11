class Graph:
    DEFAULT_WEIGHT = 1

    def neighbors(self, location):
        """Returns an iterable of locations in the neighborhood of the given
        `location` parameter.
        """
        raise NotImplementedError

    def vertices(self):
        """Returns an iterable of vertices."""
        raise NotImplementedError

    def transition_cost(self, first, second):
        """Returns the transition cost between two neighbors."""
        return self.DEFAULT_WEIGHT

class GridGraph(Graph):
    DIRECTIONS = {
            'North': (0, -1), 'South': (0, 1),
            'East': (1, 0), 'West': (-1, 0),
            'North-East': (1, -1), 'North-West': (-1, -1),
            'South-East': (1, 1), 'South-West': (-1, 1),
            }

    def __init__(self, width, height):
        """The `width` and `height` parameters should be the dimensions of the
        grid.
        """
        self.width = width
        self.height = height

    def neighbors(self, location):
        x, y = location
        for direction in self.DIRECTIONS.values():
            dx, dy = direction
            n_location = (x + dx, y + dy)
            if self.in_bounds(n_location):
                yield n_location

    def vertices(self):
        for x in range(self.width):
            for y in range(self.height):
                location = x, y
                if self.in_bounds(location) and not self.is_wall(location):
                    yield location

    def in_bounds(self, location):
        x, y = location
        return 0 <= x < self.width and 0 <= y < self.height
