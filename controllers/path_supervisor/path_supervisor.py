import json
from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

root_children = supervisor.getRoot().getField("children")

# 读取路径 JSON
with open("../../map/smoothed_path.json", "r") as f:
    data = json.load(f)

path = data["path"]

# 添加路径点（绿色+透明小球）
for point in path:
    x, y, z = point
    sphere = f'''
    Transform {{
      translation {x} {y} {z}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor 1 0 0        # 绿色
              transparency 0          # 比较透明
            }}
          }}
          geometry Sphere {{
            radius 0.1
          }}
        }}
      ]
    }}
    '''
    root_children.importMFNodeFromString(-1, sphere)

# 可以加一个循环继续运行仿真（不一定控制东西）
while supervisor.step(timestep) != -1:
    pass
