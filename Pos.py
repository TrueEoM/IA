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
        return self.m_x == other.m_x and self.m_y == other.m_y

    def __hash__(self):
        return hash(self.m_name)