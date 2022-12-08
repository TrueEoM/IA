import math
from queue import Queue
from Pos import Pos
import corredor as car
import networkx as nx
import matplotlib.pyplot as plt


class Graph:
    def __init__(self):
        self.m_map = []
        self.m_pos = []
        self.m_start = []
        self.m_ending = []
        self.m_graph = {}
        self.m_h = {}
        self.m_carros = []

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
                        if c == "X":
                            pos = Pos(x, y, c, cost=25)
                            self.m_pos.append(pos)
                            posLine.append(pos)
                        else:
                            pos = Pos(x, y, c)
                            self.m_pos.append(pos)
                            posLine.append(pos)
                            if c == "P":
                                carro = car.Corredor("c" + str(x), x, y, 0, 0)
                                self.m_start.append(pos)
                                self.m_carros.append(carro)
                            elif c == "F":
                                self.m_ending.append(pos)
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

    def get_arc_cost(self, pos1, pos2):
        cost = math.inf
        adj = self.m_graph[pos1]
        for pos in adj:
            if pos == pos2:
                cost = pos.m_cost

        return cost

    def calc_custo(self, path):
        cost = 0
        i = 0
        while i + 1 < len(path):
            cost = cost + self.get_arc_cost(path[i], path[i + 1])
            i = i + 1

        return cost

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

    def print_result(self, path):
        for posLine in self.m_map:
            out = ""
            for pos in posLine:
                if pos not in path[0]:
                    out = out + str(pos.m_type) + " "
                else:
                    out = out + "* "
            print(out)
        print("Cost: " + str(path[1]))

        pathStr = ""
        for pos in path[0]:
            pathStr = pathStr + str(pos) + " -> "

        pathStr = pathStr[:len(pathStr)-4:]
        print(pathStr)

    def desenha(self):
        # criar lista de vertices
        lista_v = self.m_pos
        g = nx.Graph()

        # Converter para o formato usado pela biblioteca networkx
        for pos in lista_v:
            n = str(pos)
            g.add_node(n)
            for adjacente in self.m_graph[pos]:
                g.add_edge(n, str(adjacente))

        # desenhar o grafo
        pos = nx.spring_layout(g)
        nx.draw_networkx(g, pos=pos, with_labels=True, font_weight='bold')
        labels = nx.get_edge_attributes(g, 'cost')
        nx.draw_networkx_edge_labels(g, pos, edge_labels=labels)

        plt.draw()
        plt.show()

    ################################################################################
    # Procura DFS [DONE]
    ################################################################################


    def ord_by_forward(self, pos, adj):
        coord = pos.get_xy()
        map_mid_x = len(self.m_map[0]) / 2
        ord = []

        if coord[0] < map_mid_x:
            for posAdj in adj:
                coordAdj = posAdj.get_xy()
                if coordAdj[0] == coord[0] + 1:
                    ord = [posAdj] + ord
                else:
                    ord.append(posAdj)
        else:
            for posAdj in adj:
                coordAdj = posAdj.get_xy()
                if coordAdj[0] == coord[0] - 1:
                    ord = [posAdj] + ord
                else:
                    ord.append(posAdj)

        return ord

    def procura_DFS(self, start, end, path=[], visited=set()):
        path.append(start)
        visited.add(start)

        if start in end:
            # calcular o custo do caminho funçao calcula custo.
            cost = self.calc_custo(path)
            return path, cost

        for adjacente in self.ord_by_forward(start, self.m_graph[start]):
            if adjacente not in visited and adjacente.m_type != "X":

                res = self.procura_DFS(adjacente, end, path, visited)
                if res is not None:
                    return res
        path.pop()  # se nao encontra remover o que está no caminho......
        return None

    ################################################################################
    # Procura Iterativa
    ################################################################################

    def procura_DFS_lim(self, start, end, limit, path=[], visited=set()):
        path.append(start)
        visited.add(start)

        if start in end:
            # calcular o custo do caminho funçao calcula custo
            cost = self.calc_custo(path)
            return path, cost

        if limit <= 0:
            return None

        for adjacente in self.ord_by_forward(start, self.m_graph[start]):
            if adjacente not in visited and adjacente.m_type != "X":
                res = self.procura_DFS_lim(adjacente, end, limit-1, path, visited)
                if res is not None:
                    return res
        path.pop()
        return None

    def procura_iterativa(self, start, end, maxDepth):
        for n in range(maxDepth):
            res = self.procura_DFS_lim(start, end, n, path=[], visited=set())
            if res is not None:
                return res

        return None

    ################################################################################
    # Procura BFS [DONE]
    ################################################################################

    def procura_BFS(self, start, end):
        # Set of visited nodes to prevent loops
        visited = set()
        queue = Queue()

        # Add the start_node to the queue and visited list
        queue.put(start)
        visited.add(start)

        # start_node has not parents
        parent = dict()
        parent[start] = None

        # Perform step 3
        path_found = False
        while not queue.empty():
            current_node = queue.get()
            if current_node in end:
                path_found = True
                break

            for next_node in self.m_graph[current_node]:
                if next_node not in visited and next_node.m_type != "X":
                    queue.put(next_node)
                    parent[next_node] = current_node
                    visited.add(next_node)

        # Path reconstruction
        path = []
        if path_found:
            if current_node in end:
                end = current_node

            path.append(end)
            while parent[end] is not None:
                path.append(parent[end])
                end = parent[end]
            path.reverse()
        return path, self.calc_custo(path)

    ################################################################################
    # Pesquisa gulosa
    ################################################################################

    def greedy(self, start, end):
        # open_list é uma lista de nodos visitados, mas com vizinhos
        # que ainda não foram todos visitados, começa com o  start
        # closed_list é uma lista de nodos visitados
        # e todos os seus vizinhos também já o foram
        open_list = set([start])
        closed_list = set([])

        # parents é um dicionário que mantém o antecessor de um nodo
        # começa com start
        parents = {}
        parents[start] = start

        while len(open_list) > 0:
            n = None

            # encontraf nodo com a menor heuristica
            for v in open_list:
                if n == None or self.m_h[v] < self.m_h[n]:
                    n = v

            if n == None:
                print('Path does not exist!')
                return None

            # se o nodo corrente é o destino
            # reconstruir o caminho a partir desse nodo até ao start
            # seguindo o antecessor
            if n == end:
                reconst_path = []

                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]

                reconst_path.append(start)

                reconst_path.reverse()

                return (reconst_path, self.calcula_custo(reconst_path))

            # para todos os vizinhos  do nodo corrente
            for (m, weight) in self.getNeighbours(n):
                # Se o nodo corrente nao esta na open nem na closed list
                # adiciona-lo à open_list e marcar o antecessor
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n

            # remover n da open_list e adiciona-lo à closed_list
            # porque todos os seus vizinhos foram inspecionados
            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None

    ################################################################################
    # Pesquisa A* (estrela)
    ################################################################################

    def pesquisa_estrela(self, start, end):
        open_list = set([start])
        closed_list = set([])

        g = {}  ##  g é apra substiruir pelo peso  ???
        g[start] = 0
        # parents contains an adjacency map of all nodes
        parents = {}
        parents[start] = start
        while len(open_list) > 0:
            n = None
            # find a node with the lowest value of f() - evaluation function
            for v in open_list:
                if n == None or g[v] + self.m_h[v] < g[n] + self.m_h[n]:  # heuristica ver.....
                    n = v;
            if n == None:
                print('Path does not exist!')
                return None

            if n == end:
                reconst_path = []
                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]
                reconst_path.append(start)
                reconst_path.reverse()
                print('Path found: {}'.format(reconst_path))
                return (reconst_path, self.calcula_custo(reconst_path))
            # for all neighbors of the current node do
            for (m, weight) in self.getNeighbours(n):  # definir função getneighbours  tem de ter um par nodo peso
                # if the current node isn't in both open_list and closed_list
                # add it to open_list and note n as it's parent
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight


                else:
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n

                        if m in closed_list:
                            closed_list.remove(m)
                            open_list.add(m)

            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None