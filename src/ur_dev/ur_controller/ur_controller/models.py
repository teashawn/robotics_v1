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

    def as_movel(self, a : float = 1.0, v : float = 1.0) -> str:
        return f"movel(p[{self.X},{self.Y},{self.Z},{self.RX},{self.RY},{self.RZ}],a={a},v={v},t=0,r=0)"

    def as_movej(self) -> str:
        return f"movej([-1.570796327,-1.570796327,-1.570796327,-1.570796327,1.570796327,0],a=5.0,v=1.0)"