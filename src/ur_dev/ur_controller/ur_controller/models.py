class Waypoint:
    def __init__(self, x, y, z, rx, ry, rz):
        # position
        self.X = x
        self.Y = y
        self.Z = z

        # orientation
        self.RX = rx
        self.RY = ry
        self.RZ = rz

    def as_movel(self, a : float = 1.0, v : float = 1.0, r : float = 0.0) -> str:
        return f"movel(p[{self.X},{self.Y},{self.Z},{self.RX},{self.RY},{self.RZ}],a={a},v={v},t=0,r={r})"

    def __repr__(self):
        return f"X: {self.X}, Y: {self.Y}, Z: {self.Z}, RX: {self.RX}, RY: {self.RY}, RZ: {self.RZ}"

    def __str__(self):
        return self.__repr__()