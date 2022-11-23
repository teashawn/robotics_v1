from ur_controller import models, constants

def _get_real_waypoints():
    return {
        "home": models.Waypoint(-0.171, -0.682, 0.428, *constants.TABLE_A_ORIENTATION),

        "box_1": models.Waypoint(0.491, -0.134, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_2": models.Waypoint(0.487, 0.042, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_3": models.Waypoint(0.491, 0.240, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_4": models.Waypoint(0.690, -0.143, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_5": models.Waypoint(0.701, 0.039, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_6": models.Waypoint(0.690, 0.230, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_7": models.Waypoint(0.888, -0.144, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_8": models.Waypoint(0.900, 0.044, -0.060, *constants.TABLE_B_ORIENTATION),
        "box_9": models.Waypoint(0.887, 0.226, -0.060, *constants.TABLE_B_ORIENTATION),

        "box_10": models.Waypoint(0.491, -0.134, 0.050, *constants.TABLE_B_ORIENTATION),
        "box_11": models.Waypoint(0.491, 0.240, 0.050, *constants.TABLE_B_ORIENTATION),
        "box_12": models.Waypoint(0.701, 0.039, 0.050, *constants.TABLE_B_ORIENTATION),
        "box_13": models.Waypoint(0.888, -0.144, 0.050, *constants.TABLE_B_ORIENTATION),
        "box_14": models.Waypoint(0.887, 0.226, 0.050, *constants.TABLE_B_ORIENTATION),

        "a_1": models.Waypoint(-0.400, -0.970, -0.080, *constants.TABLE_A_ORIENTATION),
        "a_2": models.Waypoint(-0.400, -0.280, -0.080, *constants.TABLE_A_ORIENTATION),
        "a_3": models.Waypoint(0.180, -0.280, -0.080, *constants.TABLE_A_ORIENTATION),
        "a_4": models.Waypoint(0.180, -0.970, -0.080, *constants.TABLE_A_ORIENTATION),

        "b_1": models.Waypoint(0.370, -0.280, -0.080, *constants.TABLE_B_ORIENTATION),
        "b_2": models.Waypoint(0.370, 0.410, -0.080, *constants.TABLE_B_ORIENTATION),
        "b_3": models.Waypoint(0.940, 0.410, -0.080, *constants.TABLE_B_ORIENTATION),
        "b_4": models.Waypoint(0.940, -0.280, -0.080, *constants.TABLE_B_ORIENTATION),
    }

def _get_simulated_waypoints():
    w = _get_real_waypoints()

    # patch positions of boxes that are unreachable in simulation
    w["box_9"] = w["box_11"]
    w["box_14"] = w["box_11"]

    return w

def get_waypoints(simulation : bool = True):
    if simulation:
        return _get_simulated_waypoints()
    else:
        return _get_real_waypoints()

def get_pre_pick_waypoints(waypoints, z_offset : int = 2):
    return {k: models.Waypoint(
            waypoints[k].X,
            waypoints[k].Y,
            waypoints[k].Z + (constants.BOX_SIDE*z_offset),
            *constants.TABLE_B_ORIENTATION
        ) for k in [wk for wk in waypoints.keys() if "box" in wk]}

def get_destinations(waypoints, box_spacing : float):
    return {
    "box_10": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET),
        waypoints["box_1"].Z,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_1": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 1) + box_spacing,
        waypoints["box_1"].Z,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_4": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 1) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 1) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_13": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 2) + box_spacing,
        waypoints["box_1"].Z,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_7": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 2) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 1) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_2": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 2) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 2) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_12": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_5": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 1) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_8": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 2) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_11": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 3) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),


    "box_3": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 4) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_6": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 5) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_14": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 6) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    ),
    "box_9": models.Waypoint(
        waypoints["a_4"].X - constants.EDGE_OFFSET,
        (waypoints["a_4"].Y + constants.EDGE_OFFSET) + (constants.BOX_SIDE * 3) + box_spacing,
        waypoints["box_1"].Z + (constants.BOX_SIDE * 7) + box_spacing,
        *constants.TABLE_A_ORIENTATION
    )
}

def get_pre_place_waypoints(destinations, z_offset : int = 2):
    return {k: models.Waypoint(
        destinations[k].X,
        destinations[k].Y,
        destinations[k].Z + (constants.BOX_SIDE*z_offset),
        *constants.TABLE_A_ORIENTATION
    ) for k in [wk for wk in destinations.keys() if "box" in wk]}