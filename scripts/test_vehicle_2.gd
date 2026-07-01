extends VehicleBody3D

const MAX_STEER = 0.8
const ENGINE_POWER = 50

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	steering = move_toward(steering, Input.get_axis("right", "left") * MAX_STEER, delta * 2.5)
	engine_force = Input.get_axis("backward", "forward") *  ENGINE_POWER
