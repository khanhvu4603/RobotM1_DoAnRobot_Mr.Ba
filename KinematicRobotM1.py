import math

def Forward_Kinematic(theta1, theta2, d):
    theta1 = float(theta1)
    theta2 = float(theta2)
    L1 = 65
    L2 = 170
    L3 = 160
    x = L1 + L3 * math.cos(math.radians(theta1 + theta2)) + L2 * math.cos(math.radians(theta1))
    y = L3 * math.sin(math.radians(theta1 + theta2)) + L2 * math.sin(math.radians(theta1))
    z = d
    return round(x, 2), round(y, 2), z

def Inverse_Kinematic(x, y, z):

    L1 = 65
    L2 = 170
    L3 = 160
    c2 = ((x - L1) ** 2 + y ** 2 - L3 ** 2 - L2 ** 2) / (2 * L2 * L3)
    s2 = math.sqrt(1 - c2 * c2)
    theta2 = math.atan2(s2, c2)

    c1 = ((x - L1) * (L3 * math.cos(theta2) + L2) + y * L3 * math.sin(theta2)) / \
         ((L3 * math.cos(theta2) + L2) ** 2 + (L3 * math.sin(theta2)) ** 2)
    s1 = ((L3 * math.cos(theta2) + L2) * y - (x - L1) * L3 * math.sin(theta2)) / \
         ((L3 * math.cos(theta2) + L2) ** 2 + (L3 * math.sin(theta2)) ** 2)
    theta1 = math.atan2(s1, c1)

    theta1_d = math.degrees(theta1)
    theta2_d = math.degrees(theta2)
    return round(theta1_d, 2), round(theta2_d, 2), z
    # return c2

# if __name__ == "__main__":
#     L1 = 65
#     L2 = 170
#     L3 = 110

#     while True:
#         command = input("Enter command (FORWARD or INVERSE): ").strip().upper()
#         if command == "FORWARD":
#             theta1 = float(input("Enter theta1 (degrees): "))
#             theta2 = float(input("Enter theta2 (degrees): "))
#             d = float(input("Enter d: "))
#             x, y, z = Forward_Kinematic(theta1, theta2, d)
#             print(f"X: {x}, Y: {y}, Z: {z}")
#         elif command == "INVERSE":
#             x = float(input("Enter x: "))
#             y = float(input("Enter y: "))
#             z = float(input("Enter z: "))
#             theta1, theta2, z = Inverse_Kinematic(x, y, z)
#             c2 = Inverse_Kinematic(x, y, z)
#             print(f"Theta1: {theta1}, Theta2: {theta2}, Z: {z}")
#         else:
#             print("Invalid command. Please enter FORWARD or INVERSE.")


