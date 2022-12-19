from Graph import Graph
from os.path import exists
from corredor import Corredor as car


# UI do VectorRace

def ui_jogo():
    # construção da UI
    saida = -1
    while saida != 0:
        print(" ")
        print("------------------ VectorRace v.1 ------------------")
        print(" ")
        print("1-Imprimir circuito")
        print("2-Desenhar circuito")
        print("3-Corrida com DFS")
        print("4-Corrida com BFS")
        print("5-Corrida com Iterativa")
        print("6-Corrida com Pesquisa Gulosa")
        print("7-Corrida com Pesquisa A*")
        print("0-Saír")

        saida = int(input("introduza a sua opcao-> "))

        if saida == 0:
            print("saindo.......")

        elif saida == 1:
            # Escrever o circuito como string

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()
                print(g.print_graph())
                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")


        elif saida == 2:
            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()
                g.desenha()
                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")


        elif saida == 3:
            # Efetuar corrida com DFS

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()

                if len(g.m_carros) > 1:
                    res = g.procura_DFS_multi(g.m_carros, g.m_ending)

                    for c in res:
                        print(c + ": ")
                        cost = g.calc_custo(res.get(c))
                        g.print_result_multi((res.get(c), cost), True)
                else:
                    res = g.procura_DFS(g.m_start[0], g.m_ending, path=[], visited=set())
                    g.print_result(res)

                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        elif saida == 4:
            # Efetuar corrida com BFS

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()

                if len(g.m_carros) > 1:
                    res = g.procura_BFS_multi(g.m_carros, g.m_ending)

                    for c in res:
                        print(c + ": ")
                        cost = g.calc_custo(res.get(c))
                        g.print_result_multi((res.get(c), cost), True)
                else:
                    res = g.procura_BFS(g.m_start[0], g.m_ending)
                    g.print_result(res)

                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        elif saida == 5:
            # Efetuar corrida com Iterativa

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()

                if len(g.m_carros) > 1:
                    res = None
                    i = 1
                    while res is None:
                        res = g.procura_iterativa_multi(g.m_carros, g.m_ending, i)
                        i += 1

                    for c in res:
                        print(c + ": ")
                        cost = g.calc_custo(res.get(c))
                        g.print_result_multi((res.get(c), cost), True)
                else:
                    res = None
                    i = 1
                    while res is None:
                        res = g.procura_iterativa(g.m_start[0], g.m_ending, i)
                        i += 1

                    g.print_result((res[0], len(res[0])))

                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        elif saida == 6:
            # Efetuar corrida com Greedy

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()
                g.build_heu()

                res = g.greedy(g.m_carros, g.m_ending)

                for c in res:
                    print(c + ": ")
                    g.print_result_multi((res.get(c), len(res.get(c))), True)

                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")
        elif saida == 7:
            # Efetuar corrida com A*

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()
                g.build_heu()

                res = g.procura_aStar(g.m_carros, g.m_ending)

                for c in res:
                    print(c + ": ")
                    g.print_result_multi((res.get(c), len(res.get(c))), True)

                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        else:
            print("Opção inválida...")
            l = input("prima enter para continuar")


if __name__ == "__main__":
    ui_jogo()
