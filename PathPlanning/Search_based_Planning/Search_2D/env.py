"""
Env 2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = 1024  # size of background
        self.y_range = 1547
        #expansion = 40
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        expansion = 40
        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(x): #set up walls
            obs.add((i, expansion))
        for i in range(x):
            obs.add((i, y - 1-expansion))

        for i in range(y): #set up walls
            obs.add((expansion, i))
        for i in range(y):
            obs.add((x - 1, i))

        for i in range(102-expansion, 102+366+expansion): #set up desks x
            obs.add((i, 140-expansion)) #desk 1
            obs.add((i, 140+154+expansion))
            obs.add((i, 140+154+192-expansion)) #desk 2
            obs.add((i, 140+154*2+192+expansion))
            obs.add((i, 140+154*2+192+175-expansion)) #desk 3
            obs.add((i, 140+154*3+192+175+expansion))
        for i in range(154+expansion*2):
            obs.add((102-expansion, i+140-expansion)) #desk 1
            obs.add((102+366+expansion, i+140-expansion))
            obs.add((102-expansion, i+140+154+192-expansion)) #desk 2
            obs.add((102+366+expansion, i+140+154+192-expansion))
            obs.add((102-expansion, i+140+154*2+192+175-expansion)) #desk 3
            obs.add((102+366+expansion, i+140+154*2+192+175-expansion))

        for i in range(248-expansion,248+100+expansion): #Desk 1 box (near Newton)
            obs.add((i, 140+154-expansion))
            obs.add((i, 140+154+90+expansion))
        for i in range(140+154-expansion,140+154+90+expansion): #Desk 1 box (near Newton)
            obs.add((248-expansion, i))
            obs.add((248+100+expansion, i))

        for i in range(468-expansion,468+65+expansion): #Desk 2 box
            obs.add((i, 140+154*2+192+55-expansion))
            obs.add((i, 140+154*2+192+55+35+expansion))
        for i in range(140+154*2+192+55-expansion, 140+154*2+192+55+35+expansion): #Desk 2 box
            obs.add((468-expansion, i))
            obs.add((468+65+expansion, i))

        for i in range(298-expansion,298+65+expansion): #Desk 3 box
            obs.add((i, 140+154*3+192+175-expansion))
            obs.add((i, 140+154*3+192+175+145+expansion))
        for i in range(140+154*3+192+175-expansion,140+154*3+192+175+145+expansion): #Desk 3 box
            obs.add((298-expansion, i))
            obs.add((298+65+expansion, i))

        for i in range(196-expansion,196+70+expansion): #laser box
            obs.add((i, 140+154*3+192+175+208-expansion))
            obs.add((i, 140+154*3+192+175+208+125+expansion))
        for i in range(140+154*3+192+175+208-expansion,140+154*3+192+175+208+125+expansion): #laser box
            obs.add((196-expansion, i))
            obs.add((196+70+expansion, i))

        for i in range(0,100+expansion): #laser+couch
            obs.add((i, 1072-expansion))
        for i in range(1072-expansion,1547): #laser+couch
            obs.add((100+expansion, i))

        for i in range(700-expansion,700+324): #whiteboard wall
            obs.add((i, 125-expansion))
        for i in range(125-expansion,1547): #whiteboard wall
            obs.add((700-expansion, i))

        for i in range(550-expansion,700): #material shelves
            obs.add((i, 1117-expansion))
        for i in range(1117-expansion,1547): #material shelves
            obs.add((550-expansion, i))

        for i in range(266-expansion,266+153+expansion): #printer desk
            obs.add((i, 1302-expansion))
        for i in range(1302-expansion,1547): #printer desk
            obs.add((266-expansion, i))
            obs.add((266+153+expansion, i))




        return obs
