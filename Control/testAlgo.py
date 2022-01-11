while True:
    try:
        x = int(input("Please enter a number: "))
        break
    except ValueError:
        print("Oops!  That was no valid number.  Try again...")


poses_drf = ["60","65","70"]
finger_gaps = ["34","42","50"]
shape_list = ["rectangle", "circle", "square", "hexagon"]

sqrt_hex_dim = ["20","25","30"]
circle_dim = ["20","25"]
rectangle_dim = ["20_25"]

trials = ["1","2"]

while True:
    print(("The dimension ?"))
    dim = input (rectangle_dim)
    if (dim in  rectangle_dim):
        break
    else:
        print("Oops!  That was no valid dimension.  Try again...")
