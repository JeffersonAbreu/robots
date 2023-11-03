class Tag:
    def __init__(self, name: str, pose: list[float], parent: str = None, uri: str = None):
        self.name = name
        self.pose = pose
        self.parent = parent
        self.uri = uri
    
    def getPoseValues(self) -> str:
        return "{} {} {} {} {} {}".format(*self.pose)
    
    def getKey(self):
        return int(self.name.split('_')[1]) if self.name.startswith('tag_') else None