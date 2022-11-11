from Pos import Pos


class Graph:
    def __init__(self):
        self.m_mapSize = 0
        self.m_pos = []
        self.m_graph = {}
        self.m_h = {}

    def __str__(self):
        out = ""
        for line in self.m_pos:
            for pos in line:
                out = out + str(pos) + " "
            out = out + "\n"
        return out

    def parse(self, filename):
        try:
            f = open(filename)
            Lines = f.readlines()

            x = 0
            y = 0

            for line in Lines:
                posLine = []
                for c in line:
                    if c == "X" or c == "-" or c == "F" or c == "P":
                        posLine.append(Pos(x, y, c))
                        x += 1
                self.m_pos.append(posLine)
                x = 0
                y += 1
        finally:
            f.close()

    def get_pos(self, x, y):
        for pos in self.m_pos:
            if pos.get_xy() == (x,y):
                return pos
        return None

    def neighbors(self, row_number, column_number, radius=1):
        return [[self.m_pos[i][j] if i >= 0 and i < len(self.m_pos) and j >= 0 and j < len(self.m_pos[0]) else 0
                 for j in range(column_number - 1 - radius, column_number + radius)]
                for i in range(row_number - 1 - radius, row_number + radius)]

    def make_graph(self):
        for pos in self.m_pos:
            self.m_graph[pos] = list()
            coord = pos.get_xy()
            for neighbors in self.neighbors(1, coord[0], coord[1]):
                self.m_graph[pos].append(neighbors)

    def print_graph(self):
        print(self.m_graph)