extends RigidBody3D

@export_group("Hover Settings")
@export var hover_height: float = 1.5
@export var hover_force: float = 4000.0
@export var hover_damping: float = 180.0

@export_group("Steering and Thrust")
@export var forward_thrust: float = 3000.0
@export var steering_strength: float = 450.0
@export var drift_resistance: float = 3.0 # Lateral friction factor

@export_group("Stability")
@export var alignment_torque_strength: float = 500.0
@export var alignment_damping: float = 45.0

# Raycast references
@onready var raycast_fr: RayCast3D = $RayCastFR
@onready var raycast_fl: RayCast3D = $RayCastFL
@onready var raycast_rl: RayCast3D = $RayCastRL
@onready var raycast_rr: RayCast3D = $RayCastRR

var raycasts: Array[RayCast3D] = []

# Local forward direction based on scene setup where FR/FL are at +Z
const LOCAL_FORWARD = Vector3(0, 0, 1)

func _ready() -> void:
	# Setup physics properties for the vehicle
	mass = 120.0
	gravity_scale = 1.8 
	linear_damp = 0.2
	angular_damp = 1.0
	
	raycasts = [raycast_fr, raycast_fl, raycast_rl, raycast_rr]
	
	# Configure all raycasts
	for ray in raycasts:
		ray.enabled = true
		ray.add_exception(self)
		ray.target_position = Vector3(0, -hover_height - 0.8, 0)

func _physics_process(delta: float) -> void:
	# 1. Hover Physics (4 Raycasts)
	var colliding_count = 0
	var average_normal = Vector3.ZERO
	for ray in raycasts:
		if ray.is_colliding():
			colliding_count += 1
			var collision_point = ray.get_collision_point()
			var distance = ray.global_position.distance_to(collision_point)
			var normal = ray.get_collision_normal()
			average_normal += normal
			
			# Spring-damper physics
			var displacement = hover_height - distance
			if displacement > -0.5: 
				var fraction = (hover_height - distance) / hover_height
				
				var spring_factor = fraction
				if fraction > 0.0:
					spring_factor = fraction * (1.0 + fraction * 4.0) # Quadratic scaling
				
				var spring_force_val = spring_factor * hover_force
				
				# Get velocity at this specific raycast position
				var point_vel = linear_velocity + angular_velocity.cross(ray.global_position - global_position)
				var velocity_along_up = point_vel.dot(global_transform.basis.y)
				var damper_force_val = velocity_along_up * hover_damping
				
				var total_force = max(0.0, spring_force_val - damper_force_val)
				
				# Emergency force if dangerously close to ground
				if distance < 0.4:
					var emergency_mult = (0.4 - distance) / 0.4
					total_force += emergency_mult * hover_force * 3.0
				
				# Apply force upwards relative to the ship
				var force_vector = global_transform.basis.y * total_force
				apply_force(force_vector, ray.position)
	
	# keeps the ship upright relative to the ground
	var target_up = Vector3.UP
	if colliding_count > 0:
		target_up = (average_normal / colliding_count).normalized()
	
	var current_up = global_transform.basis.y
	var error_axis = current_up.cross(target_up)
	var align_torque = error_axis * alignment_torque_strength - angular_velocity * alignment_damping
	apply_torque(align_torque)
	
	# steer
	var steer_input = 0.0
	if Input.is_key_pressed(KEY_A):
		steer_input += 1.0
	if Input.is_key_pressed(KEY_D):
		steer_input -= 1.0
		
	var yaw_torque = steer_input * steering_strength
	var global_yaw_torque = global_transform.basis.y * yaw_torque
	apply_torque(global_yaw_torque)
	
	# 4. Forward Propulsion (Spacebar)
	if Input.is_key_pressed(KEY_SPACE):
		var forward_dir = global_transform.basis * LOCAL_FORWARD
		apply_central_force(forward_dir * forward_thrust)
	
	# 5. Aerodynamic / Friction Simulation
	var local_vel = global_transform.basis.inverse() * linear_velocity
	
	# Apply drift resistance (lateral friction)
	var lateral_drag = -global_transform.basis.x * local_vel.x * drift_resistance * mass
	apply_central_force(lateral_drag)
	
	# Apply minor forward/backward drag
	var forward_dir = global_transform.basis * LOCAL_FORWARD
	var forward_drag = -forward_dir * local_vel.z * 0.12 * mass
	apply_central_force(forward_drag)
