# Name: YOUR NAME HERE

def shortest_path_grid(grid, start, goal):
    '''
    Function that returns the length of the shortest path in a 4-connected grid
    that HAS obstacles represented by 1s. The length is simply the number
    of cells on the path including the 'start' and the 'goal'

    @param grid list of lists (represents a square grid where 0 represents free space and 1s obstacles)
    @param start tuple of start index
    @param goal tuple of goal index
    @return length of path
    '''
    n = len(grid)


if __name__ == "__main__":
    grid = [[0,0,0],
            [1,1,0],
            [1,1,0]]
    start, goal = (0,1), (2,2)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 4

    grid = [[0,1],
            [1,0]]
    start, goal = (0, 0), (1,1)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == -1