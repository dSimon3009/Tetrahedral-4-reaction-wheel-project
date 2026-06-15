import bluetooth
import time
import math
import sys
from ble_simple_peripheral import BLESimplePeripheral
import MPU6050
import TB6612FNG


# Pin configuration ──────────────────────────────────────────────────

# Driver Standby
STBY1 = 16
STBY2 = 17

# Motor Offset
ofsetA1 = 1
ofsetB1 = 1
ofsetA2 = 2
ofsetB2 = 2

# Motor 1
AIN11 = 14
AIN21 = 12
# Motor 2
BIN11 = 27
BIN21 = 26
# Motor 3
AIN12 = 4
AIN22 = 33
# Motor 4
BIN12 = 0
BIN22 = 2

# PWM
PWMA1 = 13
PWMB1 = 25
PWMA2 = 32
PWMB2 = 15


# Hardware is initialized
──────────────────────────────────────────────

mpu  = MPU6050.MPU6050()
mto1 = TB6612FNG.DCMotor1(STBY1, AIN11, AIN21, PWMA1, ofsetA1)
mto2 = TB6612FNG.DCMotor2(BIN21, BIN11, STBY1, PWMB1, ofsetB1)
mto3 = TB6612FNG.DCMotor3(STBY2, AIN12, AIN22, PWMA2, ofsetA2)
mto4 = TB6612FNG.DCMotor4(BIN22, BIN12, STBY2, PWMB2, ofsetB2)


# Bluetooth ─────────────────────────────────────────────────────────

ble = bluetooth.BLE()
ble.active(True)          # ← línea que faltaba
sp  = BLESimplePeripheral(ble)


# Global variables ──────────────────────────────────────────────────────

elapsed_s        = 0.0
angular_velocity = 0.0011288          # velocidad angular orbital [rad/s] (ISS)
RadiusOrbit      = 6378000 + 400000   # radio orbital [m] (radio Tierra + altitud ISS)
engine_mode = 4 
current_theta = 0.0
position      = [RadiusOrbit, 0.0, 0.0]   # posición orbital inicial

# Inital axes
#r1 = [0,  0,  1]
#r2 = [0,  1, -1]
#r3 = [1, -1, -1]
#r4 = [-1, -1, -1]
# Axes 45 degrees rotated couterclockwise
#r1 = [0.7071, 0, 0.7071]
#r2 = [-0.7071, 1, -0.7071]
#r3 = [0.0, -1, -1.4142]
#r4 = [-1.4142, -1, 0.0]
# Axes 45 degrees rotated clockwise
#r1 = [-0.7071,  0,      0.7071]
#r2 = [ 0.7071,  1,     -0.7071]
#r3 = [ 1.4142, -1,      0.0   ]
#r4 = [ 0.0,    -1,     -1.4142]
# Axes 90 degrees rotated clockwise
#r1 = [ 0, -1,  0]
#r2 = [ 0,  1,  1]
#r3 = [ 1,  1, -1]
#r4 = [-1,  1, -1]
# Axes 90 degrees rotated counterclockwise
r1 = [ 0,  1,  0]
r2 = [ 0, -1, -1]
r3 = [ 1, -1,  1]
r4 = [-1, -1,  1]
pending_command = None


# PID Controller ─────────────────────────────────────────────────────────

class PIDController:
    def __init__(self, kp, ki, kd, dt,max_integral=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral   = 0.0
        self.prev_error = 0.0
        self.max_integral = max_integral   # ← límite anti-windup

    def compute(self, error):
        self.integral  += error * self.dt
        self.integral  = max(-self.max_integral, min(self.max_integral, self.integral))
        derivative      = (error - self.prev_error) / self.dt
        output          = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


kp = 0.1
ki = 0.075
kd = 0.5
dt = 2   #seconds

pid = PIDController(kp, ki, kd, dt)


# Callback Bluetooth ──────────────────────────────────────────────────────

def on_rx(data):
    global pending_command
    print("Dato recibido:", data)
    cmd = data.strip()
    if cmd == b"stop":
        pending_command = b"stop"
    elif cmd == b"chase":
        pending_command = b"chase"
    elif cmd == b"abort":
        pending_command = b"abort"
    elif cmd == b"mode4":         
        pending_command = b"mode4"
    elif cmd == b"mode3":          
        pending_command = b"mode3"
    else:
        print("Comando no reconocido:", cmd)
        
# Bluetooth data ──────────────────────────────────────────────────────

def send_ble(text):
    chunk = 20
    for i in range(0, len(text), chunk):
        sp.send(text[i:i+chunk])
        time.sleep_ms(30)   # esperar ACK entre fragmentos


def send_history_ble(history_separation, history_torque, history_objective, history_velocity, history_trueseparation):

    send_ble("=== RESULTS ===\n")

    send_ble("SEP(rad):\n")
    for i, v in enumerate(history_separation):
        send_ble(f"{i}:{v:.4f}\n")
        time.sleep_ms(10)

    send_ble("TORQUE:\n")
    for i, v in enumerate(history_torque):
        send_ble(f"{i}:{v:.4f}\n")
        time.sleep_ms(10)

    send_ble("DOT:\n")
    for i, v in enumerate(history_objective):
        send_ble(f"{i}:{v:.4f}\n")
        time.sleep_ms(10)
    
    send_ble("SPD:\n")
    for i, v in enumerate(history_velocity):
        send_ble(f"{i}:{v:.4f}\n")
        time.sleep_ms(10)
        
    send_ble("TSEP:\n")
    for i, v in enumerate(history_trueseparation):
        send_ble(f"{i}:{v:.4f}\n")
        time.sleep_ms(10)

    send_ble("=== END ===\n")
    
# Kalman Filter ───────────────────────────────────────────────────

class KalmanGyro:
    def __init__(self, Q=0.075, R=210.0):
        self.Q = Q        
        self.R = R       
        self.x = 0.0     
        self.P = 1.0     

    def update(self, measurement):
        # Prediction ──────────────────────────────────
        x_pred = self.x
        P_pred = self.P + self.Q

        # Correction ──────────────────────────────────
        K      = P_pred / (P_pred + self.R)

        # Update estimate
        self.x = x_pred + K * (measurement - x_pred)
        self.P = (1.0 - K) * P_pred

        return self.x


# Kalman filter for each axis
kf_x = KalmanGyro(Q=0.075, R=210.0)
kf_y = KalmanGyro(Q=0.075, R=210.0)
kf_z = KalmanGyro(Q=0.075, R=210.0)


def readgyro_filtered():
    wx, wy, wz = readgyro()
    #omega = [(omega1[0] + 7.5), (omega1[1] + 1.8), (omega1[2] - 4.1)] #for calibration
    return (
        kf_x.update(wx),
        kf_y.update(wy),
        kf_z.update(wz)
    )

# Mathematical functions ───────────────────────────────────────────────────

def normalize(v):
    mag = math.sqrt(sum(x * x for x in v))
    if mag < 1e-12:
        return [0.0] * len(v)
    return [x / mag for x in v]


def norma(v):
    return math.sqrt(sum(x * x for x in v))


def dot_product(v1, v2):
    return sum(a * b for a, b in zip(v1, v2))


def cross(v1, v2):
    return [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]
    ]


def multiplymatrices(A, B):
    m = len(A)
    k = len(B)
    n = len(B[0])
    return [[sum(A[i][p] * B[p][j] for p in range(k))
             for j in range(n)]
            for i in range(m)]


def matrixvectormultiplication(matrix, vector):
    return [sum(matrix[i][k] * vector[k] for k in range(len(vector)))
            for i in range(len(matrix))]


def transpose(M):
    rows = len(M)
    cols = len(M[0])
    return [[M[j][i] for j in range(rows)] for i in range(cols)]


def inverse(M):
    n = len(M)
    A = [row[:] for row in M]
    I = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]

    for i in range(n):
        # Partial pivot
        max_val = abs(A[i][i])
        max_row = i
        for k in range(i + 1, n):
            if abs(A[k][i]) > max_val:
                max_val = abs(A[k][i])
                max_row = k

        # If better pivot is found change rows
        if max_row != i:
            A[i], A[max_row] = A[max_row], A[i]
            I[i], I[max_row] = I[max_row], I[i]

        pivot = A[i][i]
        if abs(pivot) < 1e-12:
            raise ValueError("Singular Matrix")

        for j in range(n):
            A[i][j] /= pivot
            I[i][j] /= pivot
        for k in range(n):
            if k != i:
                factor = A[k][i]
                for j in range(n):
                    A[k][j] -= factor * A[i][j]
                    I[k][j] -= factor * I[i][j]
    return I

def wheel_allocation_matrix(A):
    LAMBDA = 1e-6   # Tikhonov

    At  = transpose(A)            # (3×4)
    AtA = multiplymatrices(At, A) # (3×3)

    # Before inverting add λI matrix is always invertible
    n = len(AtA)
    AtA_reg = [[AtA[i][j] + (LAMBDA if i == j else 0.0)
                for j in range(n)] for i in range(n)]

    AtA_inv = inverse(AtA_reg)      
    return multiplymatrices(A, AtA_inv)   # (4×3)


# Quaternion ──────────────────────────────────────────────────────────────

def quat_multiply(q1, q2):

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ]


# Sensors ────────────────────────────────────────────────────────────────

def readacceleration():
    accel = mpu.read_accel_data()
    return accel["x"], accel["y"], accel["z"]


def readgyro():
    gyro = mpu.read_gyro_data()
    return gyro["x"], gyro["y"], gyro["z"]


# Motor control ──────────────────────────────────────────────────────

def motor_off():
    mto1.stop1()
    mto2.stop2()
    mto3.stop3()
    mto4.stop4()

def motor_off_3engine():
    mto1.stop1()  
    mto2.stop2()
    mto3.stop3()
    mto4.stop4()
    
def limitvelocity(velocity):
    if velocity > 1000:
        velocity = 1000
    elif velocity < 10:
        velocity = 10
    return velocity

def engines(turnspeed):
    SCALE = 200
    DEAD  = 0.01 

    # Motor 1
    s1 = turnspeed[0] #if len(turnspeed) > 0 else 0.0
    if s1 < -DEAD:
        mto1.backwards1(int(limitvelocity(abs(s1) * SCALE)))
    elif s1 > DEAD:
        mto1.forward1(int(limitvelocity(abs(s1) * SCALE)))
    else:
        mto1.stop1()

    # Motor 2
    s2 = turnspeed[1] #if len(turnspeed) > 1 else 0.0
    if s2 < -DEAD:
        mto2.backwards2(int(limitvelocity(abs(s2) * SCALE)))
    elif s2 > DEAD:
        mto2.forward2(int(limitvelocity(abs(s2) * SCALE)))
    else:
        mto2.stop2()

    # Motor 3
    s3 = turnspeed[2] #if len(turnspeed) > 2 else 0.0
    if s3 < -DEAD:
        mto3.backwards3(int(limitvelocity(abs(s3) * SCALE)))
    elif s3 > DEAD:
        mto3.forward3(int(limitvelocity(abs(s3) * SCALE)))
    else:
        mto3.stop3()

    # Motor 4
    s4 = turnspeed[3] #if len(turnspeed) > 3 else 0.0
    if s4 < -DEAD:
        mto4.backwards4(int(limitvelocity(abs(s4) * SCALE)))
    elif s4 > DEAD:
        mto4.forward4(int(limitvelocity(abs(s4) * SCALE)))
    else:
        mto4.stop4()

def engines_3engine(turnspeed):
    SCALE = 300
    DEAD  = 0.01

    # Motor 1 
    mto1.stop1()

    # Motor 2
    s2 = turnspeed[1]
    if s2 < -DEAD:
        mto2.backwards2(int(limitvelocity(abs(s2) * SCALE)))
    elif s2 > DEAD:
        mto2.forward2(int(limitvelocity(abs(s2) * SCALE)))
    else:
        mto2.stop2()

    # Motor 3
    s3 = turnspeed[2]
    if s3 < -DEAD:
        mto3.backwards3(int(limitvelocity(abs(s3) * SCALE)))
    elif s3 > DEAD:
        mto3.forward3(int(limitvelocity(abs(s3) * SCALE)))
    else:
        mto3.stop3()

    # Motor 4
    s4 = turnspeed[3]
    if s4 < -DEAD:
        mto4.backwards4(int(limitvelocity(abs(s4) * SCALE)))
    elif s4 > DEAD:
        mto4.forward4(int(limitvelocity(abs(s4) * SCALE)))
    else:
        mto4.stop4()
# Speed distribution to reaction wheels ────────────────────────────────────

def Assignvelocities(r1_2, r2_2, r3_2, r4_2, omega_desired):

    A    = [normalize(r1_2), normalize(r2_2), normalize(r3_2), normalize(r4_2)]  # (4×3)
    W    = wheel_allocation_matrix(A)   # (4×3)
    wdot = matrixvectormultiplication(W, omega_desired)   # (4,)
    return wdot

def Assignvelocities_3engine(r2_2, r3_2, r4_2, omega_desired):

    # 3x3 matrix with only the active axes
    A = [normalize(r2_2), normalize(r3_2), normalize(r4_2)]

    # Inverse matrix
    At      = transpose(A)           # (3×3)
    AtA     = multiplymatrices(At, A) # (3×3)

    # Tikhonov
    LAMBDA  = 1e-6
    n       = len(AtA)
    AtA_reg = [[AtA[i][j] + (LAMBDA if i == j else 0.0)
                for j in range(n)] for i in range(n)]

    AtA_inv = inverse(AtA_reg)
    W       = multiplymatrices(A, AtA_inv)          # (3×3)
    w_3     = matrixvectormultiplication(W, omega_desired)  # [w2, w3, w4]

    # 4 element vector with the first position equal to 0
    return [0.0, w_3[0], w_3[1], w_3[2]]

# Update reaction wheel position ─────────────────────────────────────

def updatedmotorpositions(r1, r2, r3, r4, axis, theta):

    ax = normalize(axis)
    c  = math.cos(theta)
    s  = math.sin(theta)

    def rotate(r):
        cx = cross(ax, r)
        return [c * r[i] + s * cx[i] for i in range(3)]

    return rotate(r1), rotate(r2), rotate(r3), rotate(r4)


# Rotation quaternion───────────────────────────────────

    
def alignangleandaxis(r, target_vec):

    v1 = normalize(r)
    v2 = normalize(target_vec)

    rot_axis  = cross(v1, v2)
    sin_angle = norma(rot_axis)
    cos_angle = dot_product(v1, v2)

    if sin_angle < 1e-10:
        return [1.0, 0.0, 0.0, 0.0], 0.0

    rot_axis_n = normalize(rot_axis)
    angle      = math.atan2(sin_angle, cos_angle)
    half_angle = angle / 2.0

    quat = [
        math.cos(half_angle),
        rot_axis_n[0] * math.sin(half_angle),
        rot_axis_n[1] * math.sin(half_angle),
        rot_axis_n[2] * math.sin(half_angle)
    ]
    return quat, angle


# Detumbling ──────────────────────────────────────────────────────────────

def Detumble():
    global running
    running = True
    
    wx, wy, wz = readgyro_filtered()
    omega = [wx, wy, wz]
    w     = norma(omega)

    while abs(w) >= 9:
        print("Velocidad angular:", w, "deg/s")

        if w < 1e-12:
            break   # evitar división por cero

        # Current rotation axis
        axis = [omega[0] / w, omega[1] / w, omega[2] / w]

        # Desired angular velocity
        desired_omega = [axis[i] * (-w) for i in range(3)]

        if engine_mode == 3:
            spin = Assignvelocities_3engine(r2, r3, r4, desired_omega)
            engines_3engine(spin)
        else:
            spin = Assignvelocities(r1, r2, r3, r4, desired_omega)
            engines(spin)

        wx, wy, wz = readgyro_filtered()
        omega = [wx, wy, wz]
        w     = norma(omega)
    running = False
    motor_off()
    print("Satellite stopped, angular velocity:", w, "deg/s")


# Chase ────────────────────────────────────────────────────

def chase():

    global r1, r2, r3, r4, current_theta, position, elapsed_s, running
    running = True

    # For testing: reset conditions
    elapsed_s     = 0.0
    current_theta = 0.0
    position      = [RadiusOrbit, 0.0, 0.0]
    #r1 = [0,  0,  1]
    #r2 = [0,  1, -1]
    #r3 = [1, -1, -1]
    #r4 = [-1, -1, -1]
    #Axes 45 degrees rotated couterclockwise
    #r1 = [0.7071, 0, 0.7071]
    #r2 = [-0.7071, 1, -0.7071]
    #r3 = [0.0, -1, -1.4142]
    #r4 = [-1.4142, -1, 0.0]
    #Axes 45 degrees rotated clockwise
    #r1 = [-0.7071,  0,      0.7071]
    #r2 = [ 0.7071,  1,     -0.7071]
    #r3 = [ 1.4142, -1,      0.0   ]
    #r4 = [ 0.0,    -1,     -1.4142]
    #Axes 90 degrees rotated clockwise    
    #r1 = [ 0, -1,  0]
    #r2 = [ 0,  1,  1]
    #r3 = [ 1,  1, -1]
    #r4 = [-1,  1, -1]
    #Axes 90 degrees rotated counterclockwise     
    r1 = [ 0,  1,  0]
    r2 = [ 0, -1, -1]
    r3 = [ 1, -1,  1]
    r4 = [-1, -1,  1]
    #PID reset
    pid.integral   = 0.0
    pid.prev_error = 0.0

    # Kalman filter reset
    kf_x.x = 0.0;  kf_x.P = 1.0
    kf_y.x = 0.0;  kf_y.P = 1.0
    kf_z.x = 0.0;  kf_z.P = 1.0

    history_separation = []
    history_objective = []
    history_torque     = []
    history_velocity = []
    history_trueseparation = []

    angularvelocity = 0.0

    
    init_axis = [0.0, 0.0, 1.0]
    a, b, c, d = updatedmotorpositions(r1, r2, r3, r4, init_axis, 0.0)
    mainorientation = normalize(a)
    current_theta = angular_velocity * 0   # [rad]
    position = [
        math.cos(current_theta) * RadiusOrbit,
        math.sin(current_theta) * RadiusOrbit,
        0.0
    ]

    target = normalize(position)
    timeelapsed = 0
    # Calcular error angular inicial (en radianes)
    dp    = dot_product(mainorientation, target)
    dp    = max(-1.0, min(1.0, dp))   # clamp para evitar dominio de acos
    error = math.acos(dp)

    while error > 0.05 and timeelapsed < 100:
        
        #Abort if
        if not running:
            print("Chase interrumpido por abort.")
            motor_off()
            return [], [], [], [], []
        
        # Angular error PID control
        torque = pid.compute(error)
        time.sleep_ms(5)
        print(f"Current Torque {torque}")
        print(f"Current TARGET: {target}")
        angularvelocity += torque * dt
        wx, wy, wz = readgyro_filtered()
        omega1 = [wx, wy, wz]
        omega = [(omega1[0]), (omega1[1]), (omega1[2])] 
        w_measured    = norma(omega1)
        print(f"Rotation speed: {omega1}")
        
        physical_orientation = normalize([
            mainorientation[0] + (wx * dt * 0.01745329),
            mainorientation[1] + (wy * dt * 0.01745329),
            mainorientation[2] + (wz * dt * 0.01745329)
        ])
        physical_dp  = dot_product(physical_orientation, target)
        physical_dp  = max(-1.0, min(1.0, physical_dp))
        physical_sep = math.acos(physical_dp)        
        physical_rotation = (w_measured * 0.01745329) * dt
            
        # Calculate rotation quaternion towards objective
        quat, rotationangle = alignangleandaxis(a, target)

        # Extract rotation axis[w, x, y, z]
        rotation_axis = [quat[1], quat[2], quat[3]]
        axis_norm     = norma(rotation_axis)

        if axis_norm > 1e-6:
            rotation_axis = normalize(rotation_axis)
            rotation = physical_rotation
            # Matriz de rotación de Rodrigues
            K = [
                [0.0,              -rotation_axis[2],  rotation_axis[1]],
                [rotation_axis[2],  0.0,              -rotation_axis[0]],
                [-rotation_axis[1], rotation_axis[0],  0.0             ]
            ]
            I_mat = [[1.0 if ii == jj else 0.0 for jj in range(3)] for ii in range(3)]
            K2    = multiplymatrices(K, K)
            sin_r = math.sin(rotation)
            cos_r = math.cos(rotation)
            R = [
                [I_mat[ii][jj] + sin_r * K[ii][jj] + (1.0 - cos_r) * K2[ii][jj]
                 for jj in range(3)]
                for ii in range(3)
            ]

            mainorientation = normalize(matrixvectormultiplication(R, mainorientation))

            # Update primary axis
            a = mainorientation

            desired_omega_vec = [x * angularvelocity for x in rotation_axis]
            print(f"Antenna position: {a}")
            if engine_mode == 3:
                wdot = Assignvelocities_3engine(b, c, d, desired_omega_vec)
            else:
                wdot = Assignvelocities(a, b, c, d, desired_omega_vec)
            
            if torque > 50.0:    
                motor_off()         # If torque abmnormally high turn off engines

            # Check types
            if not all(isinstance(x, (float, int)) for x in wdot):
                print("wdot type error:", [type(x) for x in wdot])
                continue

            # Print velocities
            time.sleep_ms(10)
            print(f"E1:{wdot[0]:.4f}")
            time.sleep_ms(10)
            print(f"E2:{wdot[1]:.4f}")
            time.sleep_ms(10)
            print(f"E3:{wdot[2]:.4f}")
            time.sleep_ms(10)
            print(f"E4:{wdot[3]:.4f}")
            time.sleep_ms(10)
            print(f"Current separation: {rotationangle} radians")            

            if engine_mode == 3:
                engines_3engine(wdot)
            else:
                engines(wdot)           # activate motors
            time.sleep(2)           # for 2 seconds
            if engine_mode == 3:
                motor_off_3engine()
            else:
                motor_off()             # Turn off engines before next iteration
        else:
            print("Allignment achieved")
            break            
        # Update target and calculate error
        current_theta = angular_velocity * timeelapsed   # [rad]
        position = [
        math.cos(current_theta) * RadiusOrbit,
        math.sin(current_theta) * RadiusOrbit,
        0.0
        ]
        target = normalize(position)
        a, b, c, d = updatedmotorpositions(r1, r2, r3, r4, rotation_axis, rotation)
        r1, r2, r3, r4 = a, b, c, d
        dp     = dot_product(mainorientation, target)
        dp     = max(-1.0, min(1.0, dp))
        error  = math.acos(dp)
        print(f"Current error is  ---->  {error}")
        history_torque.append(torque)
        history_separation.append(rotationangle)
        history_objective.append(dp)
        history_velocity.append((physical_rotation))
        history_trueseparation.append(physical_sep)
        timeelapsed += 2
    if timeelapsed >= 200:
        print("Time ran out")
    elif error < 0.05 or rotationangle < 0.05:
        print("Objective reached!")
    running = False
    send_history_ble(history_separation, history_torque, history_objective, history_velocity, history_trueseparation)
    print("Historial enviado por BLE.")
    return history_separation, history_torque, history_objective, history_velocity


# Main loop ─────────────────────────────────────────────────────────

sp.on_write(on_rx)

first_iteration  = True  
was_connected    = False

while True:

    # Update Clock ──────────────────────────────────────────────────────
    t_start = time.ticks_us()
    time.sleep(2)
    t_end   = time.ticks_us()

    elapsed_s += (time.ticks_diff(t_end, t_start) / 1000000.0)

    # Orbital position ──────────────────────────────────────────────────────
    current_theta = angular_velocity * elapsed_s   # [rad]
    position = [
        math.cos(current_theta) * RadiusOrbit,
        math.sin(current_theta) * RadiusOrbit,
        0.0
    ]

    # Update motors positions ─────────────────────────────────
    if first_iteration:
        # Initial values
        #r1_main = [0,  0,  1]
        #r2_main = [0,  1, -1]
        #r3_main = [1, -1, -1]
        #r4_main = [-1, -1, -1]
        #45 degree initial position rotation
        #Axes 45 degrees rotated couterclockwise
        #r1_main = [0.7071, 0, 0.7071]
        #r2_main = [-0.7071, 1, -0.7071]
        #r3_main = [0.0, -1, -1.4142]
        #r4_main = [-1.4142, -1, 0.0]
        #Axes 45 degrees rotated clockwise
        #r1_main = [-0.7071,  0,      0.7071]
        #r2_main = [ 0.7071,  1,     -0.7071]
        #r3_main = [ 1.4142, -1,      0.0   ]
        #r4_main = [ 0.0,    -1,     -1.4142]
        #Axes 90 degrees rotated clockwise
        #r1_main = [ 0, -1,  0]
        #r2_main = [ 0,  1,  1]
        #r3_main = [ 1,  1, -1]
        #r4_main = [-1,  1, -1]
        #Axes 90 degrees rotated clockwise
        r1_main = [ 0,  1,  0]
        r2_main = [ 0, -1, -1]
        r3_main = [ 1, -1,  1]
        r4_main = [-1, -1,  1]
        first_iteration = False
    else:
        wx_main, wy_main, wz_main = readgyro_filtered()
        omega_main = [wx_main, wy_main, wz_main]
        #omega = [(omega1[0] + 7.5), (omega1[1] + 1.8), (omega1[2] - 4.1)] #for calibration
        w_main     = norma(omega_main)
        #time.sleep_ms(10)
        #print(f"Velocity around x axis:{omega[0]:.4f}")
        #time.sleep_ms(10)
        #print(f"Velocity around y axis:{omega[1]:.4f}")
        #time.sleep_ms(10)
        #print(f"Velocity around z axis:{omega[2]:.4f}")
        if w_main < 1e-10:
            w_main = 1e-10   # avoid ZeroDivisionError

        axis_main  = [omega_main[0] / w_main, omega_main[1] / w_main, omega_main[2] / w_main]
        angle_overall = w_main * 2.0   # accumulated angle [rad]

        r1_main, r2_main, r3_main, r4_main = updatedmotorpositions(r1_main, r2_main, r3_main, r4_main, axis_main, angle_overall)
        
    is_now_connected = sp.is_connected()
    if was_connected and not is_now_connected:
        pending_command = None
        print("BLE: disconnected. Re-advertising...")
        try:
            sp._advertise()
        except Exception as e:
            print("Re-advertise error:", e)
    if not was_connected and is_now_connected:
        print("BLE: client disconnected")
    was_connected = is_now_conne
    if pending_command is not None:
        cmd             = pending_command
        pending_command = None
        if cmd == b"stop":
            print("Initiating detumbling...")
            Detumble()
        elif cmd == b"chase":
            print("Initiating chase...")
            chase()
        elif cmd == b"abort":   
            abort()
        elif cmd == b"mode4":                        # ← nuevo
            engine_mode = 4
            print("Mode: 4 engines")
            send_ble("MODE: 4 motors\n")
        elif cmd == b"mode3":                        # ← nuevo
            engine_mode = 3
            mto1.stop1()
            print("Motor 1 stopped using motors 2, 3, 4.")
            send_ble("MODE: 3 motors(fM1)\n")
