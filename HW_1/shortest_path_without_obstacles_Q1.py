# Name: Adam Chehadi

def shortest_path_grid(grid, start, goal):
    '''
    Function that returns the length of the shortest path in a 4-connected
    grid that has no obstacles. The length is simply the number of cells
    on the path including the 'start' and the 'goal'.

    @param grid list of lists (represents a square grid)
    @param start tuple of start index
    @param goal tuple of goal index
    @return length of path
    '''
    n = len(grid)

    horizontal_diff = abs(goal[0] - start[0])
    vertical_diff = abs(goal[1] - start[1])

    # If start and goal in same row
    # Length = value of horizontal distance
    if(vertical_diff == 0):
        return(horizontal_diff + 1)
    # If start and goal in same column
    # Length = value of vertical distance
    elif(horizontal_diff == 0):
        return(vertical_diff + 1)
    # Else
    # Length = horizontal + vertical distance
    else:
        return(horizontal_diff + vertical_diff + 1) 


if __name__ == "__main__":
    grid = [[0,0,0],
            [0,0,0],
            [0,0,0]]
    start, goal = (0,0), (2,1)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 4

