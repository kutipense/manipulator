from math import pi

to_rad = lambda x: x/180.*pi

joint1 = {
    "number" : 1,
    "lengthX" : 0,
    "lengthY" : 0,
    "lengthZ" : 0,
    "axis" : 2,
    "angle" : 0,
    "port" : 3,
    "degree0" : 1500,
    "degree90" : 650,
    "bounds": (to_rad(-90),to_rad(90)),
}

joint2 = {
    "number" : 2,
    "lengthX" : -9.5,
    "lengthY" : 0,
    "lengthZ" : 0,
    "axis" : 1,
    "angle" : to_rad(90),
    "port" : 7,
    "degree0" : 2300,
    "degree90" : 1500,
    "bounds": (to_rad(80),to_rad(150)),
}

joint3 = {
    "number" : 3,
    "lengthX" : -10.5,
    "lengthY" : 0,
    "lengthZ" : 0,
    "axis" : 1,
    "angle" : to_rad(90),
    "port" : 15,
    "degree0" : 550,
    "degree90" : 1500,
    "bounds": (0,to_rad(150)),
}

joint4 = {
    "number" : 4,
    "lengthX" : -7.5,
    "lengthY" : 0,
    "lengthZ" : 0,
    "axis" : 1,
    "angle" : 0,
    "port" : 23,
    "degree0" : 2300,
    "degree90" : 1500,
    "bounds": (0,to_rad(110)),
}

joint5 = {
    "number" : 5,
    "lengthX" : 0,
    "lengthY" : 0,
    "lengthZ" : 0,
    "axis" : 1,
    "angle" : to_rad(90),
    "port" : 31,
    "degree0" : 2000,
    "degree90" : 1500,
    "bounds": (0,to_rad(90)),
}

