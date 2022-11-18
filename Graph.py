from Pos import Pos


class Graph:
    def __init__(self):
        self.m_pos = []
        self.m_graph = {}
        self.m_h = {}

    def __str__(self):
        out = ""
        for key in self.m_graph.keys():
            out = out + str(key) + ": " + str(self.m_graph[key]) + "\n"
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

    def neighbors(self, rowNumber, colNumber):
        result = []
        for rowAdd in range(-1, 2):
            newRow = rowNumber + rowAdd
            if newRow >= 0 and newRow <= len(self.m_pos[0]) - 1:
                for colAdd in range(-1, 2):
                    newCol = colNumber + colAdd
                    if newCol >= 0 and newCol <= len(self.m_pos) - 1:
                        if newCol == colNumber and newRow == rowNumber:
                            continue
                        result.append(self.m_pos[newCol][newRow])
        return result


    def make_graph(self):
        for posLine in self.m_pos:
            for pos in posLine:
                coord = pos.get_xy()
                for neighbors in self.neighbors(coord[0], coord[1]):
                    if pos in self.m_graph:
                        self.m_graph[pos].append(neighbors)
                    else:
                        self.m_graph[pos] = list()
                        self.m_graph[pos] = [neighbors]

    def print_graph(self):
        for key in self.m_graph.keys():
            out = ""
            for neighbour in self.m_graph.get(key):
                out = out + str(neighbour) + " "
            print("[ " + str(key) + ": " + out + "]")
