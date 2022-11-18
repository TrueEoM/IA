class Pos:
    def __init__(self, x, y, type=""):
        self.m_name = "(" + str(x) + "," + str(y) + ")"
        self.m_x    = x
        self.m_y    = y
        self.m_type = type

    def __str__(self):
        out = "(" + str(self.m_x) + "," + str(self.m_y) + "," + self.m_type + ")"
        return out

    def get_xy(self):
        return self.m_x, self.m_y

    def get_type(self):
        return self.m_type

    def __eq__(self, other):
        coordOther = other.get_xy
        return self.m_x == coordOther[0] and self.m_y == coordOther[1]

    def __hash__(self):
        return hash(self.m_name)