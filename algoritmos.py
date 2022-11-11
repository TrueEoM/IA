from queue import Queue

#biblioteca necessária para se poder utilizar o valor math.inf  (infinito)
import math

# Importar a classe nodo
#from nodo import Node

# todos os algoritmos funcionam mas precisam de ser adaptados ao contexto
# do jogo do VectorRace

################################################################################
    # Procura DFS
################################################################################


def procura_DFS(self,start, end, path=[], visited=set()):
    path.append(start)
    visited.add(start)

    if start == end:
        # calcular o custo do caminho funçao calcula custo.
        custoT= self.calcula_custo(path)
        return (path, custoT)
    for (adjacente, peso) in self.m_graph[start]:
        if adjacente not in visited:
            resultado = self.procura_DFS(adjacente, end, path, visited)
            if resultado is not None:
                return resultado
    path.pop()  # se nao encontra remover o que está no caminho......
    return None


################################################################################
    # Procura BFS
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
        if current_node == end:
            path_found = True
            break

        for (next_node, weight) in self.m_graph[current_node]:
            if next_node not in visited:
                queue.put(next_node)
                parent[next_node] = current_node
                visited.add(next_node)

    # Path reconstruction
    path = []
    if path_found:
        path.append(end)
        while parent[end] is not None:
            path.append(parent[end])
            end = parent[end]
        path.reverse()
    return (path, self.calcula_custo(path))


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

################################################################################
    # Algoritmo Minimax
################################################################################

def minimax(estado, profundidade, jogador):

    # se calhar mais adequado para modo de dois jogadores

    if jogador == BOT:
        best = [-1, -1 , -math.inf]
    else:
        best = [-1 , -1, +math.inf]

    if profundidade == 0 or (verifica_vitoria(estado, HUMANO)) or verifica_vitoria(estado, BOT):
        score = avaliar(estado)
        return [-1, -1, score]

    for call in celulas_vazias(estado):
        x, y = call[0], call[1]
        estado[x][y] = jogador
        score = minimax(estado, profundidade - 1, -jogador) # "-jogador" faz a alternação entre HUMANO e ADVERSARIO
        estado[x][y] = 0
        score[0], score[1] = x, y

        if jogador == BOT:
            if score[2] > best[2]:
                best = score

        else:
            if score[2] < best[2]:
                best = score

    return best