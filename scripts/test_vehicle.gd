extends RigidBody3D


@onready var camera_pivot: Node3D = $CameraPivot
@onready var camera_3d: Camera3D = $CameraPivot/Camera3D
@onready var reverse_camera: Camera3D = $CameraPivot/ReverseCamera


@export var wheels: Array[RayCastWheel]
@export var acceleration := 600.0
@export var deceleration := 200.0
@export var max_speed := 20.0
@export var accel_curve : Curve


var look_at
var motor_input := 0

func _unhandled_input(event: InputEvent) -> void:
	motor_input = Input.get_action_strength("forward") - Input.get_action_strength("backward")


func _ready() -> void:
	look_at = global_position


func _physics_process(delta: float) -> void:
	# camera stuff
	camera_pivot.global_position = camera_pivot.global_position.lerp(global_position, delta * 20)
	camera_pivot.transform = camera_pivot.transform.interpolate_with(transform, delta * 5.0)
	look_at = look_at.lerp(global_position + linear_velocity, delta * 5.0)
	camera_3d.look_at(look_at)
	reverse_camera.look_at(look_at)
	_check_camera_switch()
	for wheel in wheels:
		wheel.force_raycast_update()
		_do_single_wheel_suspension(wheel)
		_do_single_wheel_acceleration(wheel)


func _check_camera_switch():
	if linear_velocity.dot(transform.basis.z) > 0:
		camera_3d.current = true
	else:
		reverse_camera.current = true


func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)


func _do_single_wheel_acceleration(ray: RayCastWheel) -> void:
	var forward_dir := -ray.global_basis.z
	var vel := forward_dir.dot(linear_velocity)

	if ray.is_colliding():
		var contact := ray.wheel.global_position
		var force_pos := contact - global_position
		if ray.is_motor and motor_input:
			var speed_ratio := vel / max_speed
			var ac := accel_curve.sample_baked(speed_ratio)
			var force_vector := forward_dir * acceleration * motor_input * ac
			var projected_vector: Vector3 = (force_vector - ray.get_collision_normal() * force_vector.dot(ray.get_collision_normal()))
			apply_force(projected_vector, force_pos)
		elif abs(vel) > 0.02 and not motor_input:
			var drag_projected_vector = global_basis.z * deceleration * signf(vel)
			apply_force(drag_projected_vector, force_pos)


func _do_single_wheel_suspension(ray: RayCastWheel) -> void:
	if ray.is_colliding():
		ray.target_position.y = -(ray.rest_dist + ray.wheel_radius + ray.over_extend)
		var contact := ray.get_collision_point()
		var spring_up_dir := ray.global_transform.basis.y
		var spring_len := ray.global_position.distance_to(contact) - ray.wheel_radius
		var offset := ray.rest_dist - spring_len
		
		ray.wheel.position.y = -spring_len
		
		var spring_force := ray.spring_strength * offset
		
		var world_vel := _get_point_velocity(contact)
		var relative_vel := spring_up_dir.dot(world_vel)
		var spring_damp_force := ray.spring_damping * relative_vel
		
		var force_vector := (spring_force - spring_damp_force) * spring_up_dir
		
		contact = ray.wheel.global_position
		var force_pos_offset := contact - global_position
		apply_force(force_vector, force_pos_offset)
