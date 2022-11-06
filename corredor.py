class Corredor:

    def __init__(self, nome, p_lin, p_col, v_lin, v_col):
        # FAZER COM TUPLOS TALVEZ?
        # self.posicao = (0,0) -->  posição no circuito representado por coordenadas (linha, coluna)
        # self.velocidade = (0,0) --> velocidade no circuito representado como duas velocidades (velocidade_linha, velocidade_coluna)

        # p_lin --> posição na linha
        # p_col --> posição na coluna
        # v_lin --> velocidade na linha
        # v_col --> velocidade na coluna

        self.nome = str(nome)
        self.p_lin = p_lin
        self.p_col = p_col
        self.v_lin = v_lin
        self.v_col = v_col

    def __str__(self):
        return "Corredor " + self.nome

    def __repr__(self):
        return "Corredor " + self.nome

    def getName(self):
        return self.nome

    def setName(self, nome):
        self.nome = nome

    def __eq__(self, other):
        return self.nome == other.nome

    def __hash__(self):
        return hash(self.nome)

    def acelera_linha(self, acel_lin):
        self.p_lin = self.p_lin + self.v_lin + acel_lin
        self.v_lin = self.v_lin + acel_lin

    def acelera_coluna(self, acel_col):
        self.p_col = self.p_col + self.v_col + acel_col
        self.v_col = self.v_col + acel_col






