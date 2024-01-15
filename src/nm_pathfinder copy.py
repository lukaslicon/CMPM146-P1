from math import inf, sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = {}

    # Calculates if the point is in the given box
    def inRectangle(box, point):
        x1, x2, y1, y2 = box
        ax, ay = point
        return (ax > x1 and ax < x2 and ay > y1 and ay < y2)
    # Finds the box the point is in
    def findBox(point):
        for box in mesh['boxes']:
            if inRectangle(box, point) and box not in boxes:
                return box
    # Finds the euclidian distance
    def euclidian(a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2) * 0.5
    
    # The priority queue
    source_box = findBox(source_point)
    destination_box = findBox(destination_point)
    queue = [(0,source_box)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[source_box] = 0

    # The dictionary that will store the backpointers
    backpointers = {}
    backpointers[source_box] = None

    # The dictionary that will store the coordinates of each backpointer
    box_coord = {}
    box_coord[source_box] = source_point
    
    while queue:
        current_dist, current_box = heappop(queue)
        if current_box == destination_box:
            path = [[box_coord[current_box], destination_point]]

            # Go backwards from destination until the source using backpointers
            # and add all the nodes in the shortest path into a list
            current_back_box = backpointers[current_box]
            current_back_coord = box_coord[current_box]
            while current_back_box is not None:
                path.insert(0,[box_coord[current_back_box], current_back_coord])
                current_back_coord = box_coord[current_back_box]
                current_back_box = backpointers[current_back_box]
            return path, boxes.keys()

    return None

