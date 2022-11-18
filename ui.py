from Graph import Graph

# UI do VectorRace
# juntar ao main

def ui_jogo():

    # deve ser chamada na função main()

    #construção da UI
    saida = -1
    while saida != 0:
        print(" ")
        print("------------------ VectorRace v.1 ------------------")
        print(" ")
        print("1-Imprimir circuito")
        print("2-Desenhar circuito")
        #print("3-Imprimir  nodos de circuito")
        #print("4-Imprimir arestas de circuito")
        print("2-Corrida com DFS")
        print("3-Corrida com BFS")
        print("4-Corrida com Pesquisa Gulosa")
        print("5-Corrida com Pesquisa A*")
        print("6-Corrida com Minimax")
        print("0-Saír")

        saida = int(input("introduza a sua opcao-> "))

        if saida == 0:
            print("saindo.......")

        elif saida == 1:
            #Escrever o circuito como string

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            g.parse(filename)
            g.make_graph()
            print(g.print_graph())
            l=input("prima enter para continuar")

        elif saida == 2:
            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            g.parse(filename)
            g.make_graph()
            g.desenha()
            l = input("prima enter para continuar")

        elif saida == 3:
            #Efetuar corrida com DFS

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            g.parse(filename)
            g.make_graph()

            print(g.procura_DFS( inicio, fim, path=[], visited=set()))    # ---> precisa de ser adaptado
            l = input("prima enter para continuar")

        elif saida == 4:
            # Efetuar corrida com BFS

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            g.parse(filename)
            g.make_graph()

            print(g.procura_BFS(inicio, fim))  # ---> precisa de ser adaptado
            l = input("prima enter para continuar")

        elif saida == 5:
            # Efetuar corrida com Gulosa

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            g.parse(filename)
            g.make_graph()

            print(g.greedy(inicio, fim))  # ---> precisa de ser adaptado
            l = input("prima enter para continuar")

        elif saida == 6:
            # Efetuar corrida com A*

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            g.parse(filename)
            g.make_graph()

            print(g.pesquisa_estrela(inicio,fim))    # ---> precisa de ser adaptado
            l = input("prima enter para continuar")

        elif saida == 7:
            # Efetuar corrida com Minimax

            g = Graph()
            filename = input("insira ficheiro com circuito ---> ")
            g.parse(filename)
            g.make_graph()

            # g.minimax()

            l = input("prima enter para continuar")

        else:
            print("Opção inválida...")
            l = input("prima enter para continuar")

