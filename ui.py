from Graph import Graph
from os.path import exists


# UI do VectorRace
# juntar ao main

def ui_jogo():
    # deve ser chamada na função main()

    # construção da UI
    saida = -1
    while saida != 0:
        print(" ")
        print("------------------ VectorRace v.1 ------------------")
        print(" ")
        print("1-Imprimir circuito")
        print("2-Desenhar circuito")
        # print("3-Imprimir  nodos de circuito")
        # print("4-Imprimir arestas de circuito")
        print("3-Corrida com DFS")
        print("4-Corrida com BFS")
        print("5-Corrida com Pesquisa Gulosa")
        print("6-Corrida com Pesquisa A*")
        print("7-Corrida com Minimax")
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

                res = g.procura_BFS(g.m_start[0], g.m_ending)
                g.print_result(res)
                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        elif saida == 5:
            # Efetuar corrida com Gulosa

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()

                print(g.greedy(inicio, fim))  # ---> precisa de ser adaptado
                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        elif saida == 6:
            # Efetuar corrida com A*

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()

                print(g.pesquisa_estrela(inicio, fim))  # ---> precisa de ser adaptado
                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        elif saida == 7:
            # Efetuar corrida com Minimax

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            if (exists(filename)):
                g.parse(filename)
                g.make_graph()

                # g.minimax()

                l = input("prima enter para continuar")
            else:
                print("Ficheiro não existe!!")
                l = input("prima enter para continuar")

        else:
            print("Opção inválida...")
            l = input("prima enter para continuar")


if __name__ == "__main__":
    ui_jogo()
