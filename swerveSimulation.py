import pygame
import math

# Robot parameters
WHEEL_DISTANCE = 130  # Distance between wheels
ROBOT_WIDTH, ROBOT_HEIGHT = 160, 180
SWERVE_DIAMETER = 32
VELOCITY_SCALAR = 1  # Velocity multiplier
ROTATION_SCALAR = 0.8
ROBOT_RADIUS = 40  # Radius of the robot representation
SERVO_ROTATION_SPEED = 0.16 # sec/60 degrees
MAX_SERVO_ROTATION = 300
PIXEL_SCALAR = 10

def rotate_surface(surface, angle):
    """ Rotates a Pygame surface and resizes it to fit the new rotated dimensions. """
    # Get the original width and height
    w, h = surface.get_size()

    # Calculate the new bounding box dimensions after rotation
    diagonal = math.hypot(w, h)  # Maximum possible dimension needed
    new_size = (int(diagonal), int(diagonal))  # Ensure enough space

    # Create a larger surface with transparency to avoid clipping
    enlarged_surface = pygame.Surface(new_size, pygame.SRCALPHA)
    enlarged_surface.fill((0, 0, 0, 0))  # Fill with transparency

    # Blit the original surface to the center of the enlarged surface
    original_center = (new_size[0] // 2 - w // 2, new_size[1] // 2 - h // 2)
    enlarged_surface.blit(surface, original_center)

    # Rotate the enlarged surface
    rotated_surface = pygame.transform.rotate(enlarged_surface, angle)

    return rotated_surface

def adjust_position(robot_x, robot_y, original_surface, rotated_surface):
    """ Adjusts the draw position to keep the rotated surface centered. """
    orig_w, orig_h = original_surface.get_size()
    new_w, new_h = rotated_surface.get_size()

    # Calculate the offset needed to maintain the same center
    offset_x = (orig_w - new_w) // 2
    offset_y = (orig_h - new_h) // 2

    # Adjusted position
    new_x = robot_x + offset_x
    new_y = robot_y + offset_y

    return (new_x, new_y)

def getModuleSurface(size, angle):
    module = pygame.Surface((size, size), pygame.SRCALPHA)  # Enable transparency
    module.fill((155, 155, 155))
    
    center = size // 2
    radius = size // 2

    # Draw the circle
    pygame.draw.circle(module, (200, 200, 200), (center, center), radius)

    # Convert angle from degrees to radians
    angle_rad = math.radians(angle)

    # Calculate the endpoints of the line along the diameter
    x1 = center + radius * math.cos(angle_rad)
    y1 = center + radius * math.sin(angle_rad)
    x2 = center - radius * math.cos(angle_rad)
    y2 = center - radius * math.sin(angle_rad)

    # Draw the line
    pygame.draw.line(module, (255, 30, 50), (x1, y1), (x2, y2), 10)

    # Draw the arrowhead
    arrow_size = size // 4  # Arrow size proportional to the module
    arrow_angle = math.radians(30)  # Angle of arrowhead wings

    # Calculate arrowhead points
    arrow_x = center + (radius - arrow_size) * math.cos(angle_rad)
    arrow_y = center + (radius - arrow_size) * math.sin(angle_rad)
    
    left_x = arrow_x - arrow_size * math.cos(angle_rad - arrow_angle)
    left_y = arrow_y - arrow_size * math.sin(angle_rad - arrow_angle)
    right_x = arrow_x - arrow_size * math.cos(angle_rad + arrow_angle)
    right_y = arrow_y - arrow_size * math.sin(angle_rad + arrow_angle)

    # Draw the arrow
    pygame.draw.polygon(module, (0, 255, 0), [(arrow_x, arrow_y), (left_x, left_y), (right_x, right_y)])
    
    return module


def calculate_motor_values(target_angle):
    servo_max = MAX_SERVO_ROTATION
    
    if target_angle / servo_max > 1:
        target_position = (target_angle - 180) / servo_max
        motor_reversed = True
    else:
        target_position = target_angle / servo_max
        motor_reversed = False

    return (target_position, motor_reversed)


# Initialize pygame
pygame.init()
pygame.joystick.init()

# Screen dimensions
WIDTH, HEIGHT = 1200, 900
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2-Wheel Swerve Drive Simulation")
clock = pygame.time.Clock()



robot_x = (WIDTH // 2) - (ROBOT_WIDTH // 2)
robot_y = (HEIGHT // 2) - (ROBOT_HEIGHT // 2)
robot_angle = 0
current_servo_angle = 0
target_servo_angle = 0

# Control variables
x, y, stickAngle = 0, 0, 0
robot_x, robot_y = WIDTH // 2, HEIGHT // 2  # Robot's initial position
robot_angle = 0  # Initial angle

# Initialize joystick
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

running = True
while running:
    screen.fill((30, 30, 30))  # Clear screen
    
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    if joystick:
        x = math.trunc(joystick.get_axis(0) * 100) / 100 # Left stick X-axis
        y = -math.trunc(joystick.get_axis(1) * 100) / 100  # Left stick Y-axis (inverted for correct direction)
        stickAngle = -math.trunc(joystick.get_axis(3) * 100) / 100  # Right stick X-axis
    
    # Normalize inputs
    magnitude = math.sqrt(x**2 + y**2)
    if magnitude > 1:
        x /= magnitude
        y /= magnitude
    
    # Compute base wheel velocity
    baseWheelVelocity = math.sqrt((x*10)**2 + (y*10)**2) * 0.1
    
    # Compute robot movement angle
    if x == 0 and y == 0:
        angle = 90
    else:
        angle = math.degrees(math.atan2(y, -x)) % 360
    
    # Compute wheel velocities
    leftWheelVelocity = baseWheelVelocity - stickAngle * ROTATION_SCALAR
    rightWheelVelocity = baseWheelVelocity + stickAngle * ROTATION_SCALAR

    # Clamp values between -1 and 1
    leftWheelVelocity = max(-1, min(leftWheelVelocity, 1))
    rightWheelVelocity = max(-1, min(rightWheelVelocity, 1))

    # current angle calculations
    target_servo_angle = calculate_motor_values(angle)[0] * MAX_SERVO_ROTATION

    if calculate_motor_values(angle)[1]:
        baseWheelVelocity = -baseWheelVelocity  # Flip movement direction
        leftWheelVelocity = -leftWheelVelocity
        rightWheelVelocity = -rightWheelVelocity
    
    if (current_servo_angle < target_servo_angle):
        current_servo_angle += 60 / 60 / SERVO_ROTATION_SPEED
        if current_servo_angle > target_servo_angle:
            current_servo_angle = target_servo_angle
    elif (current_servo_angle > target_servo_angle):
        current_servo_angle -= 60 / 60 / SERVO_ROTATION_SPEED
        if current_servo_angle < target_servo_angle:
            current_servo_angle = target_servo_angle
    else:
        current_servo_angle = target_servo_angle
   
    
   # Convert angles to radians
    servo_angle_rad = math.radians(current_servo_angle)
    robot_angle_rad = math.radians(robot_angle)

    # Compute the robot's forward movement based on wheel velocity
    forward_velocity = (leftWheelVelocity + rightWheelVelocity) / 2

    # Calculate local movement relative to the robot's heading
    local_dx = (forward_velocity * math.cos(servo_angle_rad) * VELOCITY_SCALAR * PIXEL_SCALAR)
    local_dy = forward_velocity * math.sin(servo_angle_rad) * VELOCITY_SCALAR * PIXEL_SCALAR

    # Transform local movement into global coordinates
    # Transform local movement into global coordinates
    global_dx = local_dx * math.cos(robot_angle_rad) - local_dy * math.sin(robot_angle_rad)
    global_dy = local_dx * math.sin(robot_angle_rad) + local_dy * math.cos(robot_angle_rad)



    # Compute rotational velocity based on wheel difference
    rotational_velocity = (rightWheelVelocity - leftWheelVelocity) * (PIXEL_SCALAR * ROTATION_SCALAR) / WHEEL_DISTANCE
    delta_angle = rotational_velocity * 180 / math.pi  # Convert to degrees

    # Update position and angle
    robot_x += global_dx
    robot_y += global_dy

    robot_angle += delta_angle
    robot_angle %= 360

    robot = pygame.Surface((ROBOT_WIDTH, ROBOT_HEIGHT))
    robot.fill((200, 200, 200))

    pygame.draw.line(robot, (30,255,30), (0,ROBOT_HEIGHT-3), (ROBOT_WIDTH, ROBOT_HEIGHT-3), 6)

    moduleLeft = getModuleSurface(SWERVE_DIAMETER, current_servo_angle)
    moduleRight = getModuleSurface(SWERVE_DIAMETER, current_servo_angle)

    robot.blit(moduleLeft, ( (ROBOT_WIDTH//2)-(WHEEL_DISTANCE//2) - (moduleLeft.get_width()//2), ((ROBOT_HEIGHT//2) - (moduleLeft.get_width()//2))))
    robot.blit(moduleRight, ( (ROBOT_WIDTH//2)+(WHEEL_DISTANCE//2) - (moduleRight.get_width()//2), ((ROBOT_HEIGHT//2) - (moduleRight.get_width()//2))))

    robot_rotated = rotate_surface(robot, robot_angle)
    screen.blit(robot_rotated, adjust_position(robot_x, robot_y, robot, robot_rotated))

    # Display information
    font = pygame.font.Font(None, 18)
    angle_text = font.render(f"Angle: {angle:.2f}", True, (255, 255, 255))
    lw_text = font.render(f"Left Wheel power: {leftWheelVelocity:.2f}", True, (255, 255, 255))
    rw_text = font.render(f"Right Wheel power: {rightWheelVelocity:.2f}", True, (255, 255, 255))
    y_text = font.render(f"x: {x:.2f}", True, (255, 255, 255))
    x_text = font.render(f"y: {y:.2f}", True, (255, 255, 255))
    stickAngle_text = font.render(f"Stick angle: {stickAngle:.2f}", True, (255, 255, 255))
    targetAngle_text = font.render(f"Target angle: {target_servo_angle:.2f}", True, (255, 255, 255))
    currentAngle_text = font.render(f"Current angle: {current_servo_angle:.2f}", True, (255, 255, 255))
    motorReversed_text = font.render(f"Motor reversed: {calculate_motor_values(angle)[1]}", True, (255, 255, 255))
    robotHeading_text = font.render(f"Robot heading: {robot_angle:.2f}", True, (255, 255, 255))
    
    screen.blit(robotHeading_text, (10, HEIGHT-200))
    screen.blit(motorReversed_text, (10, HEIGHT-180))
    screen.blit(targetAngle_text, (10, HEIGHT-160))
    screen.blit(currentAngle_text, (10, HEIGHT-140))
    screen.blit(angle_text, (10, HEIGHT-120))
    screen.blit(lw_text, (10, HEIGHT-100))
    screen.blit(rw_text, (10, HEIGHT-80))
    screen.blit(y_text, (10, HEIGHT-60))
    screen.blit(x_text, (10, HEIGHT-40))
    screen.blit(stickAngle_text, (10, HEIGHT-20))
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
