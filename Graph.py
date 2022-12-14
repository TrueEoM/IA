import math
import random
from queue import Queue
from Pos import Pos
from Bcolor import Bcolors as bc
import corredor as car
import networkx as nx
import matplotlib.pyplot as plt
import copy


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
            carid = 1

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
                                carro = car.Corredor("c" + str(carid), pos, 0, 0)
                                carid += 1
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

    def dist(self, pos1, pos2):
        return math.dist(pos1.get_xy(), pos2.get_xy())

    def build_heu(self):
        for pos in self.m_pos:
            if pos not in self.m_ending:
                self.m_h[pos] = []
                for end in self.m_ending:
                    self.m_h[pos].append(self.dist(pos, end))
                    # self.m_h[pos].append((end, self.dist(pos, end)))
            else:
                self.m_h[pos] = []
                self.m_h[pos].append(0)

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

    def inGrid(self, pos):
        return pos.m_x < len(self.m_map[0]) and pos.m_y < len(self.m_map)

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
                    out = out + f"{path[0].index(pos) % 10} "
            print(out)

        self.cost_path_print(path)

    def print_result_multi(self, path, colorB=False):
        if colorB == True:
            color = random.randint(0, 6)

        for posLine in self.m_map:
            out = ""
            for pos in posLine:
                if pos not in path[0]:
                    out = out + str(pos.m_type) + " "
                else:
                    if color == 1:
                        out = out + f"{bc.RED}" + str(path[0].index(pos) % 10) + f"{bc.ENDC} "
                    elif color == 2:
                        out = out + f"{bc.GREEN}" + str(path[0].index(pos) % 10) + f"{bc.ENDC} "
                    elif color == 3:
                        out = out + f"{bc.YELLOW}" + str(path[0].index(pos) % 10) + f"{bc.ENDC} "
                    elif color == 4:
                        out = out + f"{bc.BLUE}" + str(path[0].index(pos) % 10) + f"{bc.ENDC} "
                    elif color == 5:
                        out = out + f"{bc.PURPLE}" + str(path[0].index(pos) % 10) + f"{bc.ENDC} "
                    elif color == 6:
                        out = out + f"{bc.CYAN}" + str(path[0].index(pos) % 10) + f"{bc.ENDC} "
                    else:
                        out = out + f"{path[0].index(pos) % 10} "
            print(out)

        self.cost_path_print(path)

    def cost_path_print(self, path):
        print("Cost: " + str(path[1]))

        pathStr = ""
        for pos in path[0]:
            pathStr = pathStr + str(pos) + " -> "

        pathStr = pathStr[:len(pathStr) - 4:]
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

    def ord_by_forward(self, start, pos, adj, multi=False):
        map_mid_x = len(self.m_map[0]) / 2

        if not multi:
            if start.m_x < map_mid_x:
                ord = sorted(adj, key=lambda e: (e.m_x, e.m_y == pos.m_y), reverse=True)
            else:
                ord = sorted(adj, key=lambda e: (e.m_x, e.m_y == pos.m_y))
        else:
            if start.m_x < map_mid_x:
                ord = sorted(adj, key=lambda e: (e.m_x, e.m_y == pos.m_y))
            else:
                ord = sorted(adj, key=lambda e: (e.m_x, e.m_y == pos.m_y), reverse=True)

        return ord

    def procura_DFS(self, start, end, path=[], visited=set()):
        path.append(start)
        visited.add(start)

        if start in end:
            cost = self.calc_custo(path)
            return path, cost

        for adjacente in self.ord_by_forward(path[0], start, self.m_graph[start]):
            if adjacente not in visited and adjacente.m_type != "X":

                res = self.procura_DFS(adjacente, end, path, visited)
                if res is not None:
                    return res

        path.pop()
        return None

    def procura_DFS_multi(self, carros, end):
        path = dict()
        visited = dict()
        stack = dict()

        for c in carros:
            stack[c.nome] = [c.pos]

        while len(stack) != 0 and len(carros) != 0:
            car_pos = []
            for c in carros:
                car_pos.append(c.pos)

            for c in carros:
                s = stack[c.nome].pop()
                while s in car_pos and s != c.pos:
                    s = stack[c.nome].pop()

                c.pos = s
                car_pos.pop(0)
                car_pos.append(c.pos)

                if c.nome in path.keys():
                    path[c.nome].append(c.pos)
                    visited[c.nome].append(c.pos)
                else:
                    path[c.nome] = [c.pos]
                    visited[c.nome] = [c.pos]

                if c.pos in end:
                    carros.remove(c)
                else:
                    for adjacente in self.ord_by_forward(path.get(c.nome)[0], c.pos, self.m_graph[c.pos], multi=True):
                        if adjacente not in visited[c.nome] and adjacente.m_type != "X":
                            stack[c.nome].append(adjacente)

        return path

    ################################################################################
    # Procura Iterativa [DONE]
    ################################################################################

    def procura_DFS_lim(self, start, end, limit, path=[], visited=set()):
        path.append(start)
        visited.add(start)

        if start in end:
            cost = self.calc_custo(path)
            return path, cost

        if limit <= 0:
            return None

        for adjacente in self.ord_by_forward(path[0], start, self.m_graph[start]):
            if adjacente not in visited and adjacente.m_type != "X":
                res = self.procura_DFS_lim(adjacente, end, limit - 1, path, visited)
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

    def procura_DFS_lim_multi(self, carros, end, limit):
        path = dict()
        visited = dict()
        stack = dict()

        for c in carros:
            stack[c.nome] = [c.pos]

        while len(stack) != 0 and len(carros) != 0:
            car_pos = []
            for c in carros:
                car_pos.append(c.pos)

            for c in carros:
                s = stack[c.nome].pop()
                while s in car_pos and s != c.pos:
                    s = stack[c.nome].pop()

                c.pos = s
                car_pos.pop(0)
                car_pos.append(c.pos)

                if c.nome in path.keys():
                    path[c.nome].append(c.pos)
                    visited[c.nome].append(c.pos)
                else:
                    path[c.nome] = [c.pos]
                    visited[c.nome] = [c.pos]

                if c.pos in end or limit <= 0:
                    carros.remove(c)
                else:
                    for adjacente in self.ord_by_forward(path.get(c.nome)[0], c.pos, self.m_graph[c.pos], multi=True):
                        if adjacente not in visited[c.nome] and adjacente.m_type != "X":
                            stack[c.nome].append(adjacente)

            limit -= 1

        for c in path:
            lastelem = path.get(c)[-1]
            if lastelem not in end:
                return None

        return path

    def procura_iterativa_multi(self, carros, end, maxDepth):
        for n in range(1, maxDepth + 1):
            res = self.procura_DFS_lim_multi(copy.deepcopy(carros), end, n)
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

            # There isn't much difference between ordering the adj. list and not doing so.
            # self.ord_by_forward(start, current_node, self.m_graph[current_node])
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

    def procura_BFS_multi(self, carros, end):
        visited = dict()
        queue = dict()
        parent = dict()
        path_found = dict()
        cfinal = []

        for c in carros:
            queue[c.nome] = Queue()
            queue[c.nome].put(c.pos)
            visited[c.nome] = set()
            visited[c.nome].add(c.pos)
            parent[c.nome] = dict()
            parent[c.nome][c.pos] = None
            path_found[c.nome] = False

        while len(queue) != 0 and len(carros) != 0:
            car_pos = []
            for c in carros:
                car_pos.append(c.pos)

            for c in carros:
                current_node = queue[c.nome].get()

                while current_node in car_pos and current_node != c.pos:
                    current_node = queue[c.nome].get()

                c.pos = current_node

                if current_node in end:
                    path_found[c.nome] = True
                    cfinal.append(copy.deepcopy(c))
                    carros.remove(c)
                    break

                car_pos.pop(0)
                car_pos.append(current_node)

                for next_node in self.m_graph[current_node]:
                    if next_node not in visited[c.nome] and next_node.m_type != "X":
                        queue[c.nome].put(next_node)
                        parent[c.nome][next_node] = current_node
                        visited[c.nome].add(next_node)

        # Path reconstruction
        path = dict()
        for c in cfinal:
            if path_found[c.nome]:
                if c.pos in end:
                    endpos = c.pos

                path[c.nome] = []
                path[c.nome].append(endpos)

                while parent[c.nome][endpos] is not None:
                    path[c.nome].append(parent[c.nome][endpos])
                    endpos = parent[c.nome][endpos]
                path[c.nome].reverse()

        return path

    ################################################################################
    # Pesquisa gulosa
    ################################################################################

    def line(self, pos1, pos2):
        res = []
        if pos1.m_y == pos2.m_y:
            for x in range(pos1.m_x, pos2.m_x):
                res.append(self.get_pos(x, pos1.m_y).m_type)
        elif pos1.m_x == pos2.m_x:
            for y in range(pos1.m_y, pos2.m_y):
                res.append(copy.deepcopy(self.get_pos(pos1.m_x, y).m_type))

        return res

    def bresenham(self, start, end):
        # step 1 get end-points of line
        (x0, y0) = start.get_xy()
        (x1, y1) = end.get_xy()
        line_type = set()

        # step 2 calculate difference
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        if dx == 0 or dy == 0:
            if y0 == y1:
                for x in range(dx):
                    if x0 < x1:
                        line_type.add(self.get_pos(x0 + x, y0).m_type)
                    else:
                        line_type.add(self.get_pos(x1 + x, y0).m_type)
            elif x0 == x1:
                for y in range(dy):
                    if y0 < y1:
                        line_type.add(copy.deepcopy(self.get_pos(x0, y + y0).m_type))
                    else:
                        line_type.add(copy.deepcopy(self.get_pos(x0, y + y1).m_type))

            return line_type
        else:
            m = dy / dx

            # step 3 perform test to check if pk < 0
            flag = True

            line_type.add(self.get_pos(x0, y0).m_type)

            step = 1
            if x0 > x1 or y0 > y1:
                step = -1

            mm = False
            if m < 1:
                x0, x1, y0, y1 = y0, y1, x0, x1
                dx = abs(x1 - x0)
                dy = abs(y1 - y0)
                mm = True

            p0 = 2 * dx - dy
            x = x0
            y = y0

            for i in range(abs(y1 - y0)):
                if flag:
                    x_previous = x0
                    p_previous = p0
                    p = p0
                    flag = False
                else:
                    x_previous = x
                    p_previous = p

                if p >= 0:
                    x = x + step

                p = p_previous + 2 * dx - 2 * dy * (abs(x - x_previous))
                y = y + 1

                if mm:
                    line_type.add(self.get_pos(y, x).m_type)
                else:
                    line_type.add(self.get_pos(x, y).m_type)

            return line_type

    def greedy(self, carros, end):
        cfinal = []
        accel = dict()
        open_list = dict()
        closed_list = dict()
        parents = dict()

        for c in carros:
            open_list[c.nome] = set()
            open_list[c.nome].add(c.pos)
            accel[c.nome] = dict()
            accel[c.nome][c.pos] = (0, 0)
            closed_list[c.nome] = set()
            parents[c.nome] = dict()
            parents[c.nome][c.pos] = None

        while len(open_list) > 0 and len(carros) > 0:
            car_pos = []
            for c in carros:
                car_pos.append(c.pos)

            for c in carros:
                best_pos = None
                a_x = a_y = 0

                for fpos in open_list[c.nome]:
                    if (best_pos is None or min(self.m_h[fpos]) < min(self.m_h[best_pos])) and fpos not in car_pos[1:]:
                        ac = accel[c.nome][fpos]
                        best_pos = fpos
                        a_x = ac[0]
                        a_y = ac[1]

                if best_pos is None:
                    carros.remove(c)

                c.acelera(a_x, a_y)
                c.pos = best_pos

                if best_pos in end:
                    c.pos = best_pos
                    cfinal.append(copy.deepcopy(c))
                    carros.remove(c)

                if self.inGrid(best_pos):
                    car_pos.pop(0)
                    car_pos.append(best_pos)

                for (x, y, ac_x, ac_y) in c.neighbours(min(self.m_h[best_pos])):
                    pos = self.get_pos(x, y)
                    if pos is not None:
                        types = self.line(pos, c.pos)  # Remove to not stop paths that collide in the x- or y-axis
                        # types = self.bresenham(c.pos, pos)
                        if pos.m_type != "X" and "X" not in types and \
                                pos not in open_list.get(c.nome) and pos not in closed_list.get(c.nome):
                            open_list[c.nome].add(pos)
                            accel[c.nome][pos] = (ac_x, ac_y)
                            parents[c.nome][pos] = best_pos

                open_list[c.nome].remove(best_pos)
                closed_list[c.nome].add(best_pos)

        if len(carros) == 0:
            path = dict()
            for c in cfinal:
                if c.pos in end:
                    endpos = c.pos

                    path[c.nome] = []
                    path[c.nome].append(endpos)

                    while parents[c.nome][endpos] is not None:
                        path[c.nome].append(parents[c.nome][endpos])
                        endpos = parents[c.nome][endpos]
                    path[c.nome].reverse()

            return path
        else:
            return None

    ################################################################################
    # Pesquisa A* (estrela)
    ################################################################################

    def procura_aStar(self, carros, end):
        open_list = dict()
        closed_list = dict()
        parents = dict()
        accel = dict()
        g = dict()
        cfinal = []

        for c in carros:
            open_list[c.nome] = set()
            open_list[c.nome].add(c.pos)
            closed_list[c.nome] = set()
            accel[c.nome] = dict()
            accel[c.nome][c.pos] = (0, 0)
            parents[c.nome] = dict()
            parents[c.nome][c.pos] = None
            g[c.nome] = dict()
            g[c.nome][c.pos] = 0

        while len(open_list) > 0 and len(carros) > 0:
            car_pos = []
            for c in carros:
                car_pos.append(c.pos)

            for c in carros:
                best_pos = None
                a_x = a_y = 0

                for pos in open_list[c.nome]:
                    if best_pos is None or (g[c.nome].get(pos) + min(self.m_h[pos]) < g[c.nome].get(best_pos)
                                            + min(self.m_h[best_pos]) and pos not in car_pos[1:]):
                        best_pos = pos
                        ac = accel[c.nome][pos]
                        a_x = ac[0]
                        a_y = ac[1]

                if best_pos is None:
                    carros.remove(c)

                c.acelera(a_x, a_y)
                c.pos = best_pos

                if best_pos in end:
                    cfinal.append(copy.deepcopy(c))
                    carros.remove(c)

                if self.inGrid(best_pos):
                    car_pos.pop(0)
                    car_pos.append(best_pos)

                for (x, y, ac_x, ac_y) in c.neighbours(min(self.m_h[best_pos])):
                    pos = self.get_pos(x, y)
                    if pos is not None:
                        types = self.line(pos, c.pos)  # Remove to not stop paths that collide in the x- or y-axis
                        # types = self.bresenham(c.pos, pos)
                        if pos.m_type != "X" and "X" not in types:
                            if pos not in open_list[c.nome] and pos not in closed_list[c.nome]:
                                open_list[c.nome].add(pos)
                                accel[c.nome][pos] = (ac_x, ac_y)
                                parents[c.nome][pos] = best_pos
                                g[c.nome][pos] = g[c.nome][best_pos] + 1
                            else:
                                if g[c.nome][pos] > g[c.nome][best_pos] + 1:
                                    g[c.nome][pos] = g[c.nome][best_pos] + 1
                                    parents[c.nome][pos] = best_pos

                                    if pos in closed_list[c.nome]:
                                        closed_list[c.nome].remove(pos)
                                        open_list[c.nome].add(pos)

                open_list[c.nome].remove(best_pos)
                closed_list[c.nome].add(best_pos)

        if len(carros) == 0:
            path = dict()
            for c in cfinal:
                if c.pos in end:
                    endpos = c.pos

                    path[c.nome] = []
                    path[c.nome].append(endpos)

                    while parents[c.nome][endpos] is not None:
                        path[c.nome].append(parents[c.nome][endpos])
                        endpos = parents[c.nome][endpos]
                    path[c.nome].reverse()

            return path
        else:
            print("All cars didn't find a path")
