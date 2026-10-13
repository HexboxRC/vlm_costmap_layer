"""Contact classification without localization or geometry inference."""
ROBOT = 'semantic_nav_robot'
OBSTACLES = frozenset(('glass_wall','wall_north','wall_south','wall_east','wall_west'))


def classify_pair(name1, name2):
    """Return ground/obstacle/irrelevant using exact scoped model names."""
    a, b = name1.split('::')[0], name2.split('::')[0]
    if a == ROBOT:
        other = b
    elif b == ROBOT:
        other = a
    else:
        return 'irrelevant'
    if other == 'ground_plane':
        return 'ground'
    return 'obstacle' if other in OBSTACLES else 'irrelevant'
