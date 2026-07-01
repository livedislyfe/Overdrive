extends RayCast3D
class_name RayCastWheel

@export var spring_strength := 100
@export var spring_damping := 2.0
@export var rest_dist := 0.5
@export var wheel_radius := 0.4
@export var over_extend := 0.0
@export var is_motor := false

@onready var wheel: Node3D = get_child(0)
