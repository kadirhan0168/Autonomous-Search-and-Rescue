from controller import Supervisor
import random

class TreeSpawner:
    def __init__(self):
        self.supervisor = Supervisor()
        self.spawn_trees(30)

    def spawn_trees(self, count):
        root = self.supervisor.getRoot()
        children = root.getField("children")
        
        for i in range(count):
            x = random.uniform(-1.5, 1.5)
            y = random.uniform(-1.5, 1.5)
            tree = f"""
            Robot {{
                translation {x} {y} 0.05
                children [
                    Solid {{
                        children [
                            DEF tree_{i} Shape {{
                                appearance PBRAppearance {{
                                    baseColor 0.435 0.306 0.216
                                    roughness 1
                                    metalness 0
                                }}
                                geometry Cylinder {{
                                    height 0.15
                                    radius 0.03
                                }}
                            }}
                        ]
                    }}
                ]
                name "tree_{i}"
                boundingObject USE tree_{i}
            }}"""
            children.importMFNodeFromString(-1, tree)

spawner = TreeSpawner()
while spawner.supervisor.step(32) != -1:
    pass