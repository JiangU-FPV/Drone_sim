from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

robot_node = supervisor.getFromDef("DRONE")
root_children = supervisor.getRoot().getField("children")

step_counter = 0
interval = 5

while supervisor.step(timestep) != -1:
    step_counter += 1
    if step_counter % interval != 0:
        continue

    position = robot_node.getPosition()

    sphere_string = f'''
    Transform {{
      translation {position[0]} {position[1]} {position[2]}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor 0 1 0  # 淡蓝色
              transparency 0.5      # 半透明，范围 0（不透明）~ 1（完全透明）
            }}
          }}
          geometry Sphere {{
            radius 0.1
          }}
        }}
      ]
    }}
    '''
    root_children.importMFNodeFromString(-1, sphere_string)
