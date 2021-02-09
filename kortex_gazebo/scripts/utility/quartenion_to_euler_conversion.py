#!/usr/bin/env python
import math
import sys
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        val_list = [roll_x, pitch_y, yaw_z]
        return val_list # in radians

def rad_to_degree(roll, pitch, yaw):

        x_deg = 180*roll/math.pi
        y_deg = 180*pitch/math.pi
        z_deg = 180*yaw/math.pi

        return x_deg, y_deg, z_deg # in degrees

if len(sys.argv) >= 5:
        x0 = float(sys.argv[1])
        y0 = float(sys.argv[2])
        z0 = float(sys.argv[3])
        w0 = float(sys.argv[4])
else:
        x0 = 0.5
        y0 = 0.5
        z0 = 0.5
        w0 = 0.5

values = euler_from_quaternion(x0, y0, z0, w0)
print("Orientation in Degrees:" + str(rad_to_degree(values[0], values[1], values[2])))
