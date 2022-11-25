from Pos import Pos
import networkx as nx
import matplotlib.pyplot as plt


class Graph:
    def __init__(self):
        self.m_map = []
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
                        self.m_pos.append(Pos(x, y, c))
                        posLine.append(Pos(x, y, c))
                        x += 1
                self.m_map.append(posLine)
                x = 0
                y += 1
        finally:
            f.close()

    def get_pos(self, x, y):
        for pos in self.m_pos:
            if pos.get_xy() == (x, y):
                return pos
        return None

    def neighbors(self, rowNumber, colNumber):
        result = []
        for rowAdd in range(-1, 2):
            newRow = rowNumber + rowAdd
            if newRow >= 0 and newRow <= len(self.m_map[0]) - 1:
                for colAdd in range(-1, 2):
                    newCol = colNumber + colAdd
                    if newCol >= 0 and newCol <= len(self.m_map) - 1:
                        if newCol == colNumber and newRow == rowNumber:
                            continue
                        result.append(self.m_map[newCol][newRow])
        return result

    def make_graph(self):
        for posLine in self.m_map:
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
            print("[ Pos " + str(key) + ": " + out + "]")

    def desenha(self):
        ##criar lista de vertices
        lista_v = self.m_pos
        lista_a = []
        g = nx.Graph()

        # Converter para o formato usado pela biblioteca networkx
        for pos in lista_v:
            n = str(pos)
            g.add_node(n)
            for adjacente in self.m_graph[pos]:
                lista = (n, str(adjacente))
                # lista_a.append(lista)
                g.add_edge(n, str(adjacente))

        # desenhar o grafo
        pos = nx.spring_layout(g)
        nx.draw_networkx(g, pos=pos, with_labels=True, font_weight='bold')
        labels = nx.get_edge_attributes(g, 'cost')
        nx.draw_networkx_edge_labels(g, pos, edge_labels=labels)

        plt.draw()
        plt.show()
