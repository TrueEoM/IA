from Pos import Pos


class Corredor:

    def __init__(self, nome, pos, v_col, v_lin):
        # p_lin --> posição na linha
        # p_col --> posição na coluna
        # v_lin --> velocidade na linha
        # v_col --> velocidade na coluna

        self.nome = str(nome)
        self.pos = pos
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

    def acelera(self, acel_lin, acel_col):
        self.pos.m_x = self.pos.m_x + self.v_lin + acel_lin
        self.v_lin = self.v_lin + acel_lin
        self.pos.m_y = self.pos.m_y + self.v_col + acel_col
        self.v_col = self.v_col + acel_col

    def neighbours(self):
        lista = []
        acel = [-1, 0, 1]

        for a_lin in acel:
            for a_col in acel:
                pos = Pos(self.pos.m_x + self.v_lin + a_lin, self.pos.m_y + self.v_col + a_col)
                lista.append([pos, a_lin, a_col])

        return lista
