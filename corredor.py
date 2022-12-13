from Pos import Pos


class Corredor:

    def __init__(self, nome, pos, v_x, v_y):
        self.nome = str(nome)
        self.pos = pos
        self.v_x = v_x
        self.v_y = v_y

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

    def calc_vel(self, a_x, a_y):
        return self.v_x + a_x, self.v_y + a_y

    def acelera(self, a_x, a_y):
        self.v_x = self.v_x + a_x
        self.v_y = self.v_y + a_y

    def neighbours(self, h):
        lista = []
        acel = [-1, 0, 1]

        for a_x in acel:
            for a_y in acel:
                p_vol = self.calc_vel(a_x, a_y)
                if abs(p_vol[0]) <= h and abs(p_vol[1]) <= h:
                    pos = (self.pos.m_x + self.v_x + a_x, self.pos.m_y + self.v_y + a_y, a_x, a_y)
                    lista.append(pos)

        return lista
