extends RigidBody3D

@export_group("Hover Settings")
@export var hover_height: float = 1.5
@export var hover_force: float = 4000.0
@export var hover_damping: float = 180.0

@export_group("Steering & Propulsion")
@export var forward_thrust: float = 3000.0
@export var steering_strength: float = 150.0
@export var auto_recenter_speed: float = 4.0
@export var mouse_sensitivity: float = 0.003
@export var drift_resistance: float = 3.0 # Lateral friction factor

@export_group("Stability")
@export var alignment_torque_strength: float = 500.0
@export var alignment_damping: float = 45.0
@export var max_bank_angle: float = 0.45 # Radian (approx 25 degrees)

@export_group("Side Thrusters")
@export var side_thruster_force: float = 1500.0

# Raycast and Marker references
@onready var raycast_fr: RayCast3D = $RayCastFR
@onready var raycast_fl: RayCast3D = $RayCastFL
@onready var raycast_rl: RayCast3D = $RayCastRL
@onready var raycast_rr: RayCast3D = $RayCastRR

@onready var fr_pos: Marker3D = $FR_Pos
@onready var fl_pos: Marker3D = $FL_Pos
@onready var rl_pos: Marker3D = $RL_Pos
@onready var rr_pos: Marker3D = $RR_Pos

var raycasts: Array[RayCast3D] = []
var rudders: Array[Node3D] = []
var steer_input: float = 0.0

# Local forward direction based on scene setup where FR/FL are at +Z
const LOCAL_FORWARD = Vector3(0, 0, 1)

func _ready() -> void:
	# Capture mouse by default for steering
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	
	# Setup physics properties for the vehicle
	mass = 120.0
	gravity_scale = 1.8 # Slightly high gravity to uphold the concept of weight
	linear_damp = 0.2
	angular_damp = 1.0
	
	raycasts = [raycast_fr, raycast_fl, raycast_rl, raycast_rr]
	
	# Configure all raycasts
	for ray in raycasts:
		ray.enabled = true
		ray.add_exception(self)
		# Set raycast direction downwards and long enough to sense the ground early
		ray.target_position = Vector3(0, -hover_height - 0.8, 0)
	
	# Find and cache rudder nodes
	_find_rudders(self)

func _find_rudders(node: Node) -> void:
	if node is Node3D and ("rudder" in node.name.to_lower() or "flap" in node.name.to_lower()):
		rudders.append(node)
	for child in node.get_children():
		_find_rudders(child)

func _input(event: InputEvent) -> void:
	# Toggle mouse capture with ESC
	if event is InputEventKey:
		if event.pressed and event.keycode == KEY_ESCAPE:
			if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
				Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
			else:
				Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

func _unhandled_input(event: InputEvent) -> void:
	# Accumulate mouse steering movement
	if event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		steer_input += event.relative.x * mouse_sensitivity
		steer_input = clamp(steer_input, -1.0, 1.0)

func _physics_process(delta: float) -> void:
	# 1. Auto-recenter steering input
	steer_input = move_toward(steer_input, 0.0, auto_recenter_speed * delta)
	
	# 2. Hover Physics (4 Raycasts)
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
			if displacement > -0.5: # Apply force even slightly above hover_height for smooth transition
				var fraction = (hover_height - distance) / hover_height
				
				# Progressive spring to prevent bottoming out under any circumstance
				var spring_factor = fraction
				if fraction > 0.0:
					spring_factor = fraction * (1.0 + fraction * 4.0) # Quadratic scaling
				
				var spring_force_val = spring_factor * hover_force
				
				# Get velocity at this specific raycast position using rigid body kinematics
				var point_vel = linear_velocity + angular_velocity.cross(ray.global_position - global_position)
				# Damp velocity along the ship's vertical axis
				var velocity_along_up = point_vel.dot(global_transform.basis.y)
				var damper_force_val = velocity_along_up * hover_damping
				
				var total_force = max(0.0, spring_force_val - damper_force_val)
				
				# Extra emergency force if dangerously close to ground (anti-hug-ground)
				if distance < 0.4:
					var emergency_mult = (0.4 - distance) / 0.4
					total_force += emergency_mult * hover_force * 3.0
				
				# Apply force upwards relative to the ship
				var force_vector = global_transform.basis.y * total_force
				apply_force(force_vector, ray.position)
	
	# 3. Alignment and Banking Stability
	var target_up = Vector3.UP
	if colliding_count > 0:
		target_up = (average_normal / colliding_count).normalized()
	
	# Bank (roll) the ship when steering
	var bank_angle = -steer_input * max_bank_angle
	var local_forward_dir = global_transform.basis * LOCAL_FORWARD
	var bank_basis = Basis(local_forward_dir.normalized(), bank_angle)
	target_up = (bank_basis * target_up).normalized()
	
	# Calculate and apply alignment torque
	var current_up = global_transform.basis.y
	var error_axis = current_up.cross(target_up)
	var align_torque = error_axis * alignment_torque_strength - angular_velocity * alignment_damping
	apply_torque(align_torque)
	
	# 4. Steering (Yaw)
	# Apply steering torque around the ship's local up axis
	var yaw_torque = -steer_input * steering_strength
	var global_yaw_torque = global_transform.basis.y * yaw_torque
	apply_torque(global_yaw_torque)
	
	# 5. Forward Propulsion (Spacebar)
	if Input.is_key_pressed(KEY_SPACE):
		var forward_dir = global_transform.basis * LOCAL_FORWARD
		apply_central_force(forward_dir * forward_thrust)
	
	# 6. Side Thruster Controls
	_handle_side_thrusters()
	
	# 7. Aerodynamic / Friction Simulation
	# Compute local velocity to split forward vs lateral movement
	var local_vel = global_transform.basis.inverse() * linear_velocity
	
	# Apply drift resistance (lateral friction)
	var lateral_drag = -global_transform.basis.x * local_vel.x * drift_resistance * mass
	apply_central_force(lateral_drag)
	
	# Apply minor forward/backward drag
	var forward_dir = global_transform.basis * LOCAL_FORWARD
	var forward_drag = -forward_dir * local_vel.z * 0.12 * mass
	apply_central_force(forward_drag)
	
	# 8. Visual Rudder Animation
	for rudder in rudders:
		rudder.rotation.y = lerp(rudder.rotation.y, steer_input * 0.52, 10.0 * delta)

func _handle_side_thrusters() -> void:
	# Define side thruster activation states
	var thrust_fl = 0.0
	var thrust_fr = 0.0
	var thrust_rl = 0.0
	var thrust_rr = 0.0
	
	# Independent keys 1-4
	if Input.is_key_pressed(KEY_1):
		thrust_fl = 1.0
	if Input.is_key_pressed(KEY_2):
		thrust_fr = 1.0
	if Input.is_key_pressed(KEY_3):
		thrust_rl = 1.0
	if Input.is_key_pressed(KEY_4):
		thrust_rr = 1.0
	
	# Left slide (Q/A) fires right side thrusters to slide Left
	if Input.is_key_pressed(KEY_Q) or Input.is_key_pressed(KEY_A):
		thrust_fr = 1.0
		thrust_rr = 1.0
	
	# Right slide (E/D) fires left side thrusters to slide Right
	if Input.is_key_pressed(KEY_E) or Input.is_key_pressed(KEY_D):
		thrust_fl = 1.0
		thrust_rl = 1.0
	
	# Apply forces at each marker if activated
	# Firing direction is always outwards relative to the ship's center line
	if thrust_fl > 0.0 and is_instance_valid(fl_pos):
		var force_dir = -sign(fl_pos.position.x) * global_transform.basis.x
		apply_force(force_dir * side_thruster_force * thrust_fl, fl_pos.position)
		
	if thrust_fr > 0.0 and is_instance_valid(fr_pos):
		var force_dir = -sign(fr_pos.position.x) * global_transform.basis.x
		apply_force(force_dir * side_thruster_force * thrust_fr, fr_pos.position)
		
	if thrust_rl > 0.0 and is_instance_valid(rl_pos):
		var force_dir = -sign(rl_pos.position.x) * global_transform.basis.x
		apply_force(force_dir * side_thruster_force * thrust_rl, rl_pos.position)
		
	if thrust_rr > 0.0 and is_instance_valid(rr_pos):
		var force_dir = -sign(rr_pos.position.x) * global_transform.basis.x
		apply_force(force_dir * side_thruster_force * thrust_rr, rr_pos.position)
